import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )
    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("dual_rm_65b_rmg24_gripper_moveit_config"),
            "config",
            "dual_rm_65b_description.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    robot_description_semantic_config = load_file(
        "dual_rm_65b_rmg24_gripper_moveit_config", "config/dual_rm_65b_description.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    kinematics_yaml = load_yaml(
        "dual_rm_65b_rmg24_gripper_moveit_config", "config/kinematics.yaml"
        )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    joint_limits_yaml = load_yaml("dual_rm_65b_rmg24_gripper_moveit_config", "config/joint_limits.yaml")
    joint_limits = {'robot_description_planning': joint_limits_yaml}
    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.8,
            "planning_time":100.0,
            "trajectory_execution/allowed_execution_duration_scaling":100
        }
    }
    ompl_planning_yaml = load_yaml(
        "dual_rm_65b_rmg24_gripper_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "dual_rm_65b_rmg24_gripper_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.sampling_frequency": 50 # 确保这个值是正数.
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            joint_limits,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
    # RViz
    rviz_base = os.path.join(get_package_share_directory("dual_rm_65b_rmg24_gripper_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )
    delayed_rviz_node = TimerAction(
        period=1.0,
        actions=[rviz_node]
    )
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "world", "dummy_base_link"],
    )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_rm_65b_rmg24_gripper_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster","left_arm_controller","right_arm_controller","platform_controller","left_rmg24_gripper_controller","right_rmg24_gripper_controller"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    # Warehouse mongodb server
    mongodb_server_node = Node(
       package="warehouse_ros_mongo",
       executable="mongo_wrapper_ros.py",
       parameters=[
           {"warehouse_port": 33829},
           {"warehouse_host": "localhost"},
           {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
       ],
       output="screen",
    )
    return LaunchDescription(
        [
            tutorial_arg,
            delayed_rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        # mongodb_server_node,
        ]
        #+ load_controllers
    )

