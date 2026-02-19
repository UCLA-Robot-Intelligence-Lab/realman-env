# RealMan Robot Launcher Guide

## Quick Start
```bash
# Launch arms only (working)
./launch_robot.sh

# Stop: Press Ctrl+C
```

## Arms Status ✅
- **Left Arm**: Port 8089, IP 169.254.128.20
- **Right Arm**: Port 8090, IP 169.254.128.20
- **Controller**: RM65-BI dual arms
- **Status**: Working perfectly

## Camera Issue ❌
- **Problem**: class_loader symbol lookup error
- **Cause**: Library version mismatch (ROS 2 Foxy vs realsense2_camera)
- **Solutions**:
  1. Upgrade to ROS 2 Humble
  2. Rebuild class_loader from source
  3. Use USB cameras instead

## ML Environment 🤖
```bash
# Activate conda environment
conda activate real

# Test LeRobot
python -c "import lerobot; print('LeRobot:', lerobot.__version__)"

# Test PyTorch
python -c "import torch; print('PyTorch:', torch.__version__)"
```

## File Structure
```
launch_robot.sh          # Main launcher (推荐使用)
launch_arms_only.py      # Arms-only Python script
launch_drivers_fixed.py  # Fixed version with camera info
launch_drivers.py         # Original broken version
```

## ROS 2 Commands
```bash
# Check running nodes
ros2 node list

# Check arm topics
ros2 topic list | grep arm

# Check joint states
ros2 topic echo /joint_states
```

## Troubleshooting
- **Arms not connecting**: Check network connection to 169.254.128.20
- **Cameras failing**: Known issue, use arms-only launcher
- **ML imports not working**: Activate conda environment with `conda activate real`