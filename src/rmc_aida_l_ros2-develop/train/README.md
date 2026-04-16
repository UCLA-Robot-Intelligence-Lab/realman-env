# train

Training scripts for imitation learning and vision-language-action (VLA) models.
All scripts use LeRobot v3 datasets and save checkpoints under `models/`.

Set `REPO_ID` (and other config vars) at the top of each script before running.

---

## Imitation Learning

### ACT — `imitation/train_act.py`
**Action Chunking with Transformers.** Predicts a chunk of future actions at once.
Best for smooth, precise manipulation. Fast training, modest VRAM.

Key config:
```python
REPO_ID        = "user/dataset"
TRAINING_STEPS = 100_000
BATCH_SIZE     = 8
LR             = 1e-5
CHUNK_SIZE     = 100   # future steps predicted per forward pass
```

### Diffusion Policy — `imitation/train_diffusion.py`
**Denoising diffusion for action generation.** Handles multimodal distributions well.
Uses observation history (`n_obs_steps=3`).

Key config:
```python
REPO_ID        = "user/dataset"
TRAINING_STEPS = 100_000
BATCH_SIZE     = 16
LR             = 1e-4
```

### VQ-BeT — `imitation/train_vq_bet.py`
**Vector-Quantized Behavior Transformer.** Discretizes action space via VQ-VAE,
then uses a GPT-style transformer. Good for multimodal/diverse behaviors.

Key config:
```python
REPO_ID        = "user/dataset"
TRAINING_STEPS = 100_000
BATCH_SIZE     = 16
LR             = 1e-4
```

---

## Vision-Language-Action (VLA)

VLA models require a large GPU (20–40+ GB VRAM).

### SmolVLA — `vla/train_smolvla.py`
Compact VLA model. Conditions on language + images → actions. ~20GB VRAM.

### Pi0-Fast — `vla/train_pi0fast.py`
Physical Intelligence's fast inference VLA. Flow-matching action head. ~40GB VRAM.

### Pi0.5 — `vla/train_pi0_5.py`
Enhanced Pi0 with better generalization. PaliGemma backbone. ~40GB VRAM.

### xVLA — `vla/train_xvla.py`
Cross-embodiment VLA. May not be in public LeRobot yet — the script gracefully
falls back with a message if unavailable.

All VLA scripts share the same config pattern:
```python
REPO_ID        = "user/dataset"
TRAINING_STEPS = 50_000
BATCH_SIZE     = 4
LR             = 2e-5
```

---

## Output

Each run saves to `models/<model>_YYYYMMDD_HHMMSS/`:
```
models/act_20250101_120000/
├── checkpoints/
│   ├── step_010000/
│   └── step_020000/
└── final/          ← point deploy.py POLICY_PATH here
```
