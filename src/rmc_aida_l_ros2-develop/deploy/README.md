# deploy

Run a trained LeRobot policy on the real robot.

---

## Usage

**Requires:** arm drivers running (`launch_drivers.py`) and conda env active.

Edit config at top of `deploy.py`:
```python
POLICY_PATH = "./train/imitation/models/<run>/final"
FPS         = 10
ONE_ARM     = None    # None, "left", or "right"
BASE        = False
PLATFORM    = False
```
```bash
python deploy.py
```

Press `Enter` to start an episode, `Ctrl+C` to exit.

---

## How it works

```
get_state() + get_images()
        │
        ▼
  build_obs_batch()        BGR→RGB, CHW, normalized
        │
        ▼
  policy.select_action()   runs on GPU if available
        │
        ▼
  bridge.send_action()     rad→deg for arms, 0–1→servo for hands
```

The bridge connects to both arms via TCP (`bridge.connect_arms()`) before the loop starts.
The `ONE_ARM`, `BASE`, `PLATFORM` flags must match how the policy was trained.

---

## Notes

- `POLICY_PATH` should point to a directory containing `config.json` + model weights saved by `save_pretrained()`.
- The action vector layout must match the state vector layout from training.
- Base commands publish to `/cmd_vel` as `geometry_msgs/Twist`.
- Platform control is not yet implemented.
