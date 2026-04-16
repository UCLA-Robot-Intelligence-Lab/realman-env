# data_collection/core

Core modules shared by `collect.py` and `deploy/deploy.py`.

---

## Modules

### `hardware_bridge.py` — `RealmanHardwareBridge`

Unified ROS2 node for reading robot state and sending commands.

- **State reading:** ROS2 subscriptions (joint states, hand status, arm pose, odometry)
- **Command sending:** Robotic_Arm Python API over TCP (only needed for deployment)

```python
bridge = RealmanHardwareBridge(one_arm=None, base=False, platform=False)

# read-only (data collection)
state  = bridge.get_state()   # float32 array, shape=(state_dim,)
images = bridge.get_images()  # {camera_name: (H,W,C) uint8 BGR}

# read + write (deployment)
bridge.connect_arms()
bridge.send_action(action_vector)   # same layout as get_state()
bridge.disconnect_arms()
```

**ROS2 topics subscribed:**

| Topic | Message | Data |
|-------|---------|------|
| `/joint_states` | `sensor_msgs/JointState` | Arm joint angles (rad) |
| `/left_arm_controller/rm_driver/udp_hand_status` | `Handstatus` | Left hand finger positions (0–1000) |
| `/right_arm_controller/rm_driver/udp_hand_status` | `Handstatus` | Right hand finger positions |
| `/left_arm_controller/rm_driver/udp_arm_position` | `geometry_msgs/Pose` | Left arm end-effector pose |
| `/right_arm_controller/rm_driver/udp_arm_position` | `geometry_msgs/Pose` | Right arm end-effector pose |
| `/odom` | `nav_msgs/Odometry` | Base velocity (only when `base=True`) |

---

### `lerobot_recorder.py` — `LeRobotRecorder`

Wraps `LeRobotDataset` to record episodes in v3 format (Parquet + MP4).

```python
recorder = LeRobotRecorder(
    repo_id="user/dataset",
    root="./collected_data",
    fps=10,
    state_names=bridge.state_names,
    camera_names=["top", "left_wrist"],
)
recorder.start_episode()
recorder.add_frame(obs=state, action=action, images=images)
recorder.end_episode(task="pick and place")
recorder.finalize()         # must call before push_to_hub
recorder.push_to_hub()      # optional
```

---

### `camera_utils.py` — `AsyncCameraReader`

Reads Intel RealSense D435 cameras asynchronously using `pyrealsense2`.
Returns BGR frames as numpy arrays. Camera serials are defined in `constants.py`.

---

### `constants.py`

All hardware constants in one place — IPs, ports, topics, DOF, camera serials, defaults.
Edit here when hardware changes (e.g. different arm model, new camera serial, different speed).

Key values:

| Constant | Value | Notes |
|----------|-------|-------|
| `LEFT_ARM_IP` | `169.254.128.18` | |
| `RIGHT_ARM_IP` | `169.254.128.19` | |
| `ARM_PORT` | `8080` | |
| `ARM_DOF` | `6` | Change to 7 for RM75-B |
| `HAND_DOF` | `6` | Fingers per hand |
| `HAND_SERVO_MAX` | `1000` | TODO: verify with hardware |
| `ARM_SPEED` | `20` | Deployment joint speed (deg/s) |
