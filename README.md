# realman-env

ROS2 workspace for the **RealMan AIDA** dual-arm composite lifting robot.

**Hardware:** RM65-B dual arms · 6-DOF dexterous hands · Intel RealSense D435 cameras · Woosh AGV base · lift platform
**Software:** ROS2 Foxy · LeRobot ≥ 0.4.0 · PyTorch ≥ 2.0

---

## Environment Setup

Two Python environments are required — keep them separate:

```bash
# Terminal 1: ROS2 drivers (system Python 3.8)
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# Terminal 2: ML stack (conda)
conda activate realman-env
pip install lerobot torch
```

---

## Workflow

```
1. Launch drivers     →  data_collection/launch_drivers.py
2. Collect data       →  data_collection/collect.py
3. Train              →  train/imitation/ or train/vla/
4. Deploy             →  deploy/deploy.py
```

### 1. Launch drivers
Edit `ARM_MODEL` / `ARM_ONLY` at top of the file, then:
```bash
# system Python 3.8
python3.8 src/rmc_aida_l_ros2-develop/data_collection/launch_drivers.py
```

### 2. Collect data
Edit config vars at top of `collect.py` (`REPO_ID`, `TASK`, `FPS`, `ONE_ARM`, etc.), then:
```bash
# conda env
python src/rmc_aida_l_ros2-develop/data_collection/collect.py
```

### 3. Train
Edit `REPO_ID` (and other vars) at top of the script, then run it:
```bash
# imitation learning
python train/imitation/train_act.py
python train/imitation/train_diffusion.py
python train/imitation/train_vq_bet.py

# vision-language-action (large GPU required)
python train/vla/train_smolvla.py
python train/vla/train_pi0fast.py
python train/vla/train_pi0_5.py
```

### 4. Deploy
Edit `POLICY_PATH`, `ONE_ARM`, `BASE`, `PLATFORM` at top of `deploy.py`, then:
```bash
python src/rmc_aida_l_ros2-develop/deploy/deploy.py
```

---

## State Vector Layout

| Config               | Dim | Layout                                        |
|----------------------|-----|-----------------------------------------------|
| `ONE_ARM = None`     | 24  | `[l_arm×6, l_hand×6, r_arm×6, r_hand×6]`    |
| `ONE_ARM = "left"`   | 12  | `[l_arm×6, l_hand×6]`                        |
| `ONE_ARM = "right"`  | 12  | `[r_arm×6, r_hand×6]`                        |
| `BASE = True`        | +3  | append `[vx, vy, omega]`                     |
| `PLATFORM = True`    | +1  | append `[height]`                            |

---

## Robot IPs

| Component | IP              | Port |
|-----------|-----------------|------|
| Left arm  | 169.254.128.18  | 8080 |
| Right arm | 169.254.128.19  | 8080 |

---

## Package Structure

```
src/rmc_aida_l_ros2-develop/
├── data_collection/    # drivers, data collection, LeRobot recording
├── deploy/             # policy deployment on real hardware
├── train/              # imitation learning + VLA training scripts
└── ros2_*/             # ROS2 driver packages (arm, camera, AGV)
```

See each subfolder's `README.md` for details.
