#!/bin/bash

echo "=== RealMan Robot Launcher ==="
echo "1. Launching ROS 2 Arms (system Python 3.8)..."
echo "2. Conda environment available for ML/LeRobot"
echo ""

# Launch arms with system Python 3.8
cd /home/rm/realman-env/src/rmc_aida_l_ros2-develop/data_collection
python3.8 launch_arms_only.py &
ROS_PID=$!

echo ""
echo "✅ Arms launched with PID: $ROS_PID"
echo "📹 Cameras disabled (class_loader compatibility issue)"
echo ""
echo "🤖 ML Environment:"
echo "   conda activate real"
echo "   python -c 'import lerobot; print(lerobot.__version__)'"
echo ""
echo "Press Ctrl+C to stop"

# Wait for Ctrl+C
trap 'echo "Stopping ROS drivers..."; kill $ROS_PID; exit' INT
wait