#!/bin/bash

# ROS 2 + LeRobot launcher script
# Uses system Python 3.8 for ROS, conda environment for ML parts

echo "RealMan Robot Launcher"
echo "======================"

# ROS 2 Environment (system Python 3.8)
echo "Setting up ROS 2 Foxy environment..."
source /opt/ros/foxy/setup.bash
source /home/rm/ros2_ws/install/setup.bash

# Launch ROS 2 drivers with system Python
echo "Launching ROS 2 drivers with system Python 3.8..."
cd /home/rm/realman-env/src/rmc_aida_l_ros2-develop/data_collection
python3.8 launch_drivers_fixed.py &

ROS_PID=$!

# Conda Environment (for ML/LeRobot)
echo "Setting up conda environment for ML..."
source ~/miniconda3/etc/profile.d/conda.sh
conda activate realman-env

echo "Conda environment activated. Use:"
echo "  python -c 'import lerobot; print(lerobot.__version__)'"  
echo "  python -c 'import torch; print(torch.cuda.is_available())'"
echo ""
echo "ROS drivers running with PID: $ROS_PID"
echo "Press Ctrl+C to stop both"

# Wait for Ctrl+C
trap 'echo "Stopping..."; kill $ROS_PID; exit' INT
wait