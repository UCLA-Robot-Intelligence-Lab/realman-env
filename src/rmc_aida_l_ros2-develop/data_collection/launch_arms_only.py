import os
from ament_index_python.packages import get_package_share_directory as get_pkg
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription as Include
from launch.launch_description_sources import PythonLaunchDescriptionSource as Source

def generate():
    # Only launch arm drivers - skip cameras due to class_loader conflicts
    arm_path = os.path.join(get_pkg('rm_driver'), 'launch', 'dual_rm_65_driver.launch.py')
    
    return LaunchDescription([
        Include(Source(arm_path)),
        # Cameras disabled due to class_loader symbol lookup errors
        # *[Include(Source(rs_path), launch_arguments={'camera_name': f'camera{i}'}.items()) for i in range(1, 4)]
    ])

if __name__ == '__main__':
    print("Launching Arms Only (Cameras disabled due to library conflicts)...")
    print("Arms configured on ports 8089 (left) and 8090 (right) at IP 169.254.128.20")
    print("To fix cameras: upgrade to ROS 2 Humble or rebuild class_loader from source")
    
    ls = LaunchService()
    ls.include_launch_description(generate())
    ls.run()