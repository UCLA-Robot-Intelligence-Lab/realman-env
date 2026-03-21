#!/usr/bin/env python3
"""
Train xVLA on a RealMan dataset.

xVLA (Cross-embodiment VLA) is designed for transfer across robot embodiments.
NOTE: xVLA may not yet be available in the public LeRobot package.
      Check https://github.com/huggingface/lerobot for availability.

If xVLA is not available, use train_smolvla.py or train_pi0fast.py instead.

Usage:
    python train_xvla.py [--repo-id user/my-dataset]
"""

from datetime import datetime
from pathlib import Path

import torch

REPO_ID         = "repo_id_here"
TRAINING_STEPS  = 50_000
CHECKPOINT_FREQ = 5_000
BATCH_SIZE      = 4
LR              = 2e-5


def main():
    # Try to import xVLA — fall back with a helpful message if not available
    try:
        from lerobot.policies.xvla.configuration_xvla import XVLAConfig
        from lerobot.policies.xvla.modeling_xvla import XVLAPolicy
        PolicyConfig = XVLAConfig
        PolicyClass  = XVLAPolicy
    except ImportError:
        print("xVLA not found in lerobot. Check https://github.com/huggingface/lerobot for updates.")
        print("Alternatives: train_smolvla.py  train_pi0fast.py  train_pi0_5.py")
        return

    from lerobot.configs.types import FeatureType
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
    from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset
    from lerobot.datasets.utils import dataset_to_policy_features
    from lerobot.policies.factory import make_pre_post_processors

    run_dir = Path("models") / datetime.now().strftime("xvla_%Y%m%d_%H%M%S")
    ckpt_dir  = run_dir / "checkpoints"
    final_dir = run_dir / "final"
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    final_dir.mkdir(parents=True, exist_ok=True)
    print(f"Saving to: {run_dir}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    dataset_metadata = LeRobotDatasetMetadata(REPO_ID)
    features = dataset_to_policy_features(dataset_metadata.features)
    output_features = {k: ft for k, ft in features.items() if ft.type is FeatureType.ACTION}
    input_features  = {k: ft for k, ft in features.items() if k not in output_features}

    cfg    = PolicyConfig(input_features=input_features, output_features=output_features)
    policy = PolicyClass(cfg, dataset_stats=dataset_metadata.stats)
    policy.train()
    policy.to(device)
    preprocessor, postprocessor = make_pre_post_processors(cfg, dataset_stats=dataset_metadata.stats)

    obs_idx = getattr(cfg, "observation_delta_indices", None)
    act_idx = getattr(cfg, "action_delta_indices", None)
    delta_timestamps = {}
    for key in input_features:
        delta_timestamps[key] = [i / dataset_metadata.fps for i in obs_idx] if obs_idx else [0.0]
    delta_timestamps["action"] = [i / dataset_metadata.fps for i in act_idx] if act_idx else [0.0]

    dataset   = StreamingLeRobotDataset(REPO_ID, delta_timestamps=delta_timestamps, tolerance_s=1e-3)
    optimizer = torch.optim.AdamW(policy.parameters(), lr=LR, weight_decay=1e-2)
    dataloader = torch.utils.data.DataLoader(
        dataset, batch_size=BATCH_SIZE, num_workers=4,
        pin_memory=device.type != "cpu", drop_last=True,
    )

    step, done = 0, False
    while not done:
        for batch in dataloader:
            batch = preprocessor(batch)
            batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}
            loss, _ = policy.forward(batch)
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()

            if step % 10 == 0:
                print(f"step {step} | loss {loss.item():.4f}")

            if step > 0 and step % CHECKPOINT_FREQ == 0:
                out = ckpt_dir / f"step_{step:06d}"
                out.mkdir(parents=True, exist_ok=True)
                policy.save_pretrained(out)
                preprocessor.save_pretrained(out)
                postprocessor.save_pretrained(out)
                print(f"Checkpoint: {out}")

            step += 1
            if step >= TRAINING_STEPS:
                done = True
                break

    policy.save_pretrained(final_dir)
    preprocessor.save_pretrained(final_dir)
    postprocessor.save_pretrained(final_dir)
    print(f"Done. Final model: {final_dir}")


if __name__ == "__main__":
    main()
