#!/usr/bin/env python3
"""
Train VQ-BeT (Vector-Quantized Behavior Transformer) on a RealMan dataset.

VQ-BeT discretizes the action space via VQ-VAE, then uses a GPT-style
transformer to predict action tokens — handles multimodal behavior well.

Usage:
    python train_vq_bet.py [--repo-id user/my-dataset] [--steps 100000]
"""

from datetime import datetime
from pathlib import Path

import torch

from lerobot.configs.types import FeatureType
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset
from lerobot.datasets.utils import dataset_to_policy_features
from lerobot.policies.vqbet.configuration_vqbet import VQBeTConfig
from lerobot.policies.vqbet.modeling_vqbet import VQBeTPolicy
from lerobot.policies.factory import make_pre_post_processors

REPO_ID         = "repo_id_here"
TRAINING_STEPS  = 100_000
CHECKPOINT_FREQ = 10_000
BATCH_SIZE      = 16
LR              = 1e-4


def main():
    run_dir = Path("models") / datetime.now().strftime("vqbet_%Y%m%d_%H%M%S")
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

    cfg = VQBeTConfig(input_features=input_features, output_features=output_features)

    policy = VQBeTPolicy(cfg, dataset_stats=dataset_metadata.stats)
    policy.train()
    policy.to(device)
    preprocessor, postprocessor = make_pre_post_processors(cfg, dataset_stats=dataset_metadata.stats)

    delta_timestamps = {}
    for key in input_features:
        delta_timestamps[key] = [i / dataset_metadata.fps for i in cfg.observation_delta_indices]
    delta_timestamps["action"] = [i / dataset_metadata.fps for i in cfg.action_delta_indices]

    dataset   = StreamingLeRobotDataset(REPO_ID, delta_timestamps=delta_timestamps, tolerance_s=1e-3)
    optimizer = torch.optim.Adam(policy.parameters(), lr=LR)
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
