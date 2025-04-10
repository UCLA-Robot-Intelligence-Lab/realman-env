#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace gazebo
{
  class MimicJointPlugin : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    std::vector<std::pair<physics::JointPtr, physics::JointPtr>> jointPairs;
    std::vector<double> multipliers;
    std::vector<double> offsets;

  public:
    MimicJointPlugin() : ModelPlugin() {}
    
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      model = _model;

      // Load all mimic joints and their properties
      if (_sdf->HasElement("drive_joint"))
      {
        std::string driveJointName = _sdf->Get<std::string>("drive_joint");
        physics::JointPtr driveJoint = model->GetJoint(driveJointName);
        
        if (!driveJoint)
        {
          RCLCPP_ERROR(rclcpp::get_logger("MimicJointPlugin"), "Drive joint [%s] not found in model [%s]",
                       driveJointName.c_str(), model->GetName().c_str());
          return;
        }

        sdf::ElementPtr mimicElem = _sdf->GetElement("mimic_joint");
        while (mimicElem)
        {
          std::string mimicJointName = mimicElem->Get<std::string>("name");
          physics::JointPtr mimicJoint = model->GetJoint(mimicJointName);
          double multiplier = mimicElem->Get<double>("multiplier");
          double offset = mimicElem->Get<double>("offset");

          if (mimicJoint)
          {
            jointPairs.push_back(std::make_pair(driveJoint, mimicJoint));
            multipliers.push_back(multiplier);
            offsets.push_back(offset);
          }
          else
          {
            RCLCPP_ERROR(rclcpp::get_logger("MimicJointPlugin"), "Mimic joint [%s] not found in model [%s]",
                         mimicJointName.c_str(), model->GetName().c_str());
          }

          mimicElem = mimicElem->GetNextElement("mimic_joint");
        }
      }

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MimicJointPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      for (size_t i = 0; i < jointPairs.size(); ++i)
      {
        double drivePosition = jointPairs[i].first->Position(0);
        jointPairs[i].second->SetPosition(0, drivePosition * multipliers[i] + offsets[i]);
      }
    }

  private:
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)
}
