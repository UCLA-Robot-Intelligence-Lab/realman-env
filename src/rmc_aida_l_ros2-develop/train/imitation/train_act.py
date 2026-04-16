#!/usr/bin/env python3
"""
Train ACT (Action Chunking with Transformers) on a RealMan LeRobot dataset.

ACT predicts a fixed-length chunk of future actions at once, which reduces
compounding errors and produces smoother motion — well-suited for manipulation.

Edit the config variables below, then run:
    python train_act.py
"""

from datetime import datetime
from pathlib import Path

import torch

from lerobot.configs.types import FeatureType
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset
from lerobot.datasets.utils import dataset_to_policy_features
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors

REPO_ID         = "repo_id_here"
TRAINING_STEPS  = 100_000
CHECKPOINT_FREQ = 10_000
BATCH_SIZE      = 8
LR              = 1e-5
CHUNK_SIZE      = 100   # future action steps ACT predicts at once


def main():
    run_dir = Path("models") / datetime.now().strftime("act_%Y%m%d_%H%M%S")
    checkpoints_dir = run_dir / "checkpoints"
    final_dir = run_dir / "final"
    checkpoints_dir.mkdir(parents=True, exist_ok=True)
    final_dir.mkdir(parents=True, exist_ok=True)
    print(f"Saving to: {run_dir}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # --- dataset features ---
    dataset_metadata = LeRobotDatasetMetadata(REPO_ID)
    features = dataset_to_policy_features(dataset_metadata.features)
    output_features = {k: ft for k, ft in features.items() if ft.type is FeatureType.ACTION}
    input_features  = {k: ft for k, ft in features.items() if k not in output_features}

    # --- policy config ---
    cfg = ACTConfig(
        input_features=input_features,
        output_features=output_features,
        chunk_size=CHUNK_SIZE,
        n_obs_steps=1,
    )

    policy = ACTPolicy(cfg, dataset_stats=dataset_metadata.stats)
    policy.train()
    policy.to(device)
    preprocessor, postprocessor = make_pre_post_processors(cfg, dataset_stats=dataset_metadata.stats)

    # ACT: n_obs_steps=1 so observation_delta_indices is None — use current frame only.
    # action_delta_indices = list(range(chunk_size))
    delta_timestamps = {}
    for key in input_features:
        delta_timestamps[key] = [0.0]
    delta_timestamps["action"] = [i / dataset_metadata.fps for i in range(cfg.chunk_size)]

    dataset   = StreamingLeRobotDataset(REPO_ID, delta_timestamps=delta_timestamps, tolerance_s=1e-3)
    optimizer = torch.optim.Adam(policy.parameters(), lr=LR)
    dataloader = torch.utils.data.DataLoader(
        dataset, batch_size=BATCH_SIZE, num_workers=4,
        pin_memory=device.type != "cpu", drop_last=True,
    )

    # --- training loop ---
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
                ckpt = checkpoints_dir / f"step_{step:06d}"
                ckpt.mkdir(parents=True, exist_ok=True)
                policy.save_pretrained(ckpt)
                preprocessor.save_pretrained(ckpt)
                postprocessor.save_pretrained(ckpt)
                print(f"Checkpoint saved: {ckpt}")

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
