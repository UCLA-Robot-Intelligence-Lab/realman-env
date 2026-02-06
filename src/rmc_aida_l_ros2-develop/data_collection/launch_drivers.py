import os
from ament_index_python.packages import get_package_share_directory as get_pkg
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription as Include
from launch.launch_description_sources import PythonLaunchDescriptionSource as Source

def generate():
    rs_path = os.path.join(get_pkg('realsense2_camera'), 'launch', 'rs_launch.py')
    arm_path = os.path.join(get_pkg('rm_driver'), 'launch', 'dual_rm_65_driver.launch.py')
    
    return LaunchDescription([
        Include(Source(arm_path)),
        *[Include(Source(rs_path), launch_arguments={'camera_name': f'camera{i}'}.items()) for i in range(1, 4)]
    ])

if __name__ == '__main__':
    print("Launching Arms & 3 Cameras...")
    ls = LaunchService()
    ls.include_launch_description(generate())
    ls.run()
