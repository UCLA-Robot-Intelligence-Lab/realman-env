#!/usr/bin/env python3
"""
Upload a locally recorded LeRobot dataset to HuggingFace Hub.

Usage:
    python upload_to_hf.py \\
        --repo-id  your-username/my-dataset \\
        --root     ./collected_data
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", required=True, help="HuggingFace repo id (user/dataset-name)")
    parser.add_argument("--root",    required=True, help="Local dataset directory")
    parser.add_argument("--private", action="store_true", help="Create as private repo")
    args = parser.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    print(f"Loading dataset from {args.root}...")
    dataset = LeRobotDataset(repo_id=args.repo_id, root=args.root)

    print(f"Episodes: {dataset.num_episodes}  |  Frames: {len(dataset)}")
    print(f"Pushing to Hub: {args.repo_id}...")

    dataset.push_to_hub(private=args.private)
    print("Done.")


if __name__ == "__main__":
    sys.exit(main())
