# RealMan Dual-Arm Data Collection System

Clean, modular data collection pipeline for RealMan dual-arm robot with 6-DOF hands and RealSense cameras.

## 🏗️ Architecture

### Core Components

1. **`hardware_bridge_clean.py`** - Clean ROS2 hardware bridge
   - Subscribes to joint states, arm poses, and 6-DOF hand status
   - Provides unified robot state (24D: left_arm(6) + left_hand(6) + right_arm(6) + right_hand(6))
   - Integrated camera reader for RealSense D435 cameras

2. **`data_recorder.py`** - Synchronized data recorder
   - Records robot state and camera data at specified FPS
   - Saves images and robot state with synchronized timestamps
   - Threaded recording for real-time performance

3. **`meta_recorder.py`** - Episode and dataset metadata
   - Tracks episodes, tasks, and dataset statistics
   - Manages dataset configuration and environment info
   - Provides dataset summaries and episode management

4. **`collect_data.py`** - Main data collection script
   - Integrates all components
   - Command-line and interactive modes
   - Handles system initialization and cleanup

### Supporting Files

- **`constants.py`** - System constants and configuration
- **`camera_utils.py`** - RealSense camera utilities
- **`test_*.py`** - Test scripts for individual components

## 🚀 Quick Start

### 1. Interactive Mode (Recommended for testing)
```bash
cd /home/rm/realman-env/src/rmc_aida_l_ros2-develop/data_collection/core
source /opt/ros/foxy/setup.bash
/usr/bin/python3.8 collect_data.py --interactive
```

### 2. Automated Collection
```bash
# Collect for 60 seconds with task name
/usr/bin/python3.8 collect_data.py --task "demo_task" --duration 60 --fps 10

# Custom dataset name and output directory
/usr/bin/python3.8 collect_data.py --dataset "my_dataset" --output "data" --duration 120
```

### 3. Individual Component Testing
```bash
# Test hardware bridge
/usr/bin/python3.8 hardware_bridge_clean.py

# Test data recorder
/usr/bin/python3.8 data_recorder.py

# Test meta recorder
/usr/bin/python3.8 meta_recorder.py
```

## 📊 Data Structure

### Output Directory Structure
```
collected_data/
├── metadata.json                    # Dataset metadata
├── recording_summary.json           # Recording summary
├── top_camera_XXXXXX.png            # Top camera images
├── left_wrist_XXXXXX.png            # Left wrist camera images
├── right_wrist_XXXXXX.png           # Right wrist camera images
└── robot_state_XXXXXX.json          # Robot state data
```

### Robot State Format (24D)
```
[left_arm(6), left_hand(6), right_arm(6), right_hand(6)]
```

- **Arm joints**: 6 DOF per arm (radians)
- **Hand positions**: 6 DOF per hand, normalized 0-1 (0=closed, 1=open)

### Camera Data
- **Resolution**: 640x480 @ 30fps
- **Format**: BGR8 color images
- **Cameras**: 2x Intel RealSense D435

## 🤖 Robot Configuration

### RealMan Dual-Arm System
- **Arms**: Left + Right, 6 DOF each
- **Hands**: 6-DOF dexterous hands (not grippers)
- **Hand Data**: angles, positions, states, forces per finger

### Topics Monitored
- `/joint_states` - Unified joint states
- `/left_arm_controller/rm_driver/udp_arm_position` - Left arm pose
- `/right_arm_controller/rm_driver/udp_arm_position` - Right arm pose
- `/left_arm_controller/rm_driver/udp_hand_status` - Left hand status
- `/right_arm_controller/rm_driver/udp_hand_status` - Right hand status

## 📷 Camera System

### Intel RealSense D435
- **Camera 0**: Serial `027322070274` (top_camera)
- **Camera 1**: Serial `216322071814` (left_wrist)
- **Resolution**: 640x480
- **Streams**: Color + Depth (depth saved but not used in state)

## 🎯 Usage Examples

### Interactive Session
```bash
$ /usr/bin/python3.8 collect_data.py --interactive
🎮 Interactive Data Collection Mode
Commands: 'start', 'stop', 'status', 'info', 'quit'

> start
Task name (optional): pick_and_place
Task description (optional): Pick up object and place in target
Duration in seconds (optional): 30

🔴 Data collection started (Episode: episode_0001_1234567890)
📹 Recorded 50 frames (5.0s elapsed)
📹 Recorded 100 frames (10.0s elapsed)
...
✅ Recording completed: 300 frames
📊 Stats: {'frames_recorded': 300, 'recording': False, ...}

> stop
Success? (y/n): y
Notes (optional): Successful demonstration
✅ Ended episode: episode_0001_1234567890
   Duration: 30.0s, Frames: 300

📊 Dataset: realman_dataset
📁 Location: collected_data
📈 Episodes: 1
🖼 Frames: 300
⏱ Duration: 0.01 hours
🎬 Avg Episode: 30.0s
📷 Avg FPS: 10.0
💾 Size: 45.2 MB
🎯 Tasks: pick_and_place
```

### Command Line Collection
```bash
# Quick 30-second recording
/usr/bin/python3.8 collect_data.py --task "demo" --duration 30

# Long recording with custom settings
/usr/bin/python3.8 collect_data.py --dataset "research_data" --fps 5 --duration 300
```

## 🔧 Configuration

### Constants (`constants.py`)
```python
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
ARM_SPEED = 0.5  # rad/s
DEFAULT_COLLECTION_RATE = 10  # Hz
```

### Customization
- Modify `constants.py` for system parameters
- Extend `hardware_bridge_clean.py` for additional robot data
- Update `camera_utils.py` for different camera configurations

## 🐛 Troubleshooting

### Common Issues
1. **ROS2 Import Errors**: Use Python 3.8 with proper ROS2 sourcing
2. **Camera Not Found**: Check RealSense connections and permissions
3. **No Robot Data**: Verify robot drivers are running and topics are published

### Debug Commands
```bash
# Check ROS2 topics
ros2 topic list | grep -E "(joint|arm|hand)"

# Check camera devices
/usr/bin/python3.8 -c "import pyrealsense2; print('RealSense available')"

# Test individual components
/usr/bin/python3.8 hardware_bridge_clean.py
```

## 📚 Integration with LeRobot

The collected data is ready for LeRobot training:
- **State format**: 24D robot state vector
- **Image format**: Standard PNG images
- **Metadata**: JSON format with episode information
- **Synchronization**: Timestamped data for temporal alignment

## 📄 License

RealMan Robotics - Data Collection Pipeline