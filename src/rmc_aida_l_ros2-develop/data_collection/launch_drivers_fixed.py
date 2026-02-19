import os
import sys
from ament_index_python.packages import get_package_share_directory as get_pkg

def generate_launch_description():
    """Generate launch description for arm drivers only (cameras disabled due to library conflicts)"""
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription as Include
    from launch.launch_description_sources import PythonLaunchDescriptionSource as Source
    
    arm_path = os.path.join(get_pkg('rm_driver'), 'launch', 'dual_rm_65_driver.launch.py')
    
    return LaunchDescription([
        Include(Source(arm_path)),
    ])

if __name__ == '__main__':
    print("Launching Arms (Cameras disabled due to library conflicts)...")
    print("To enable cameras, fix class_loader compatibility or upgrade to ROS 2 Humble")
    
    # Try to use launch with ROS 2 environment
    try:
        from launch import LaunchService
        ls = LaunchService()
        ls.include_launch_description(generate_launch_description())
        ls.run()
    except ImportError as e:
        print(f"ROS 2 launch error: {e}")
        print("Please run with: source /opt/ros/foxy/setup.bash && python3.8 launch_drivers_fixed.py")