# data_collection

Collects kinesthetic teaching demonstrations and saves them as a LeRobot v3 dataset (Parquet + MP4).

---

## Files

| File                | Description |
|---------------------|-------------|
| `launch_drivers.py` | Launch ROS2 arm + camera drivers (system Python 3.8) |
| `collect.py`        | Interactive episode recorder — saves to LeRobot format |
| `upload_to_hf.py`   | Push a saved dataset to HuggingFace Hub |
| `core/`             | Hardware bridge, recorder, camera utils, constants |

---

## Usage

### Step 1 — Launch drivers (system Python 3.8)

Edit config at top of `launch_drivers.py`:
```python
ARM_MODEL = "rm65"   # or "rm75"
ARM_ONLY  = False    # True = skip cameras
```
```bash
python3.8 launch_drivers.py
```

### Step 2 — Collect data (conda env)

Edit config at top of `collect.py`:
```python
REPO_ID    = "your-username/my-dataset"
OUTPUT_DIR = "./collected_data"
TASK       = "pick and place"
FPS        = 10
EPISODES   = 0       # 0 = unlimited
ONE_ARM    = None    # None, "left", or "right"
BASE       = False
PLATFORM   = False
```
```bash
python collect.py
```

Press `Enter` to start/stop each episode. Press `Ctrl+C` to finish and finalize.

### Step 3 — Upload (optional)

Edit `upload_to_hf.py` and run:
```bash
python upload_to_hf.py
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

## Notes

- During collection, `action = state` (kinesthetic teaching — the policy learns to imitate what the operator does).
- Teleop scripts live separately and are responsible for moving the arms during recording.
- The bridge reads state via ROS2 subscriptions — no arm API connection needed for collection.
