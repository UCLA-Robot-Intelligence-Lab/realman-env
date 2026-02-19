from pathlib import Path
from datetime import datetime
import torch

from lerobot.configs.types import FeatureType
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset
from lerobot.datasets.utils import dataset_to_policy_features
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

REPO_ID = "repo_id_here"
TRAINING_STEPS = 100000
CHECKPOINT_FREQ = 10000


def main():
    # -------------------------------------------------
    # Run directory with timestamp
    # -------------------------------------------------
    run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir = Path("models") / run_name
    checkpoints_dir = run_dir / "checkpoints"
    final_dir = run_dir / "final"

    checkpoints_dir.mkdir(parents=True, exist_ok=True)
    final_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Saving outputs to: {run_dir}")

    # device selection
    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    print(f"[INFO] Using device: {device}")

    log_freq = 10

    # --- dataset metadata
    dataset_metadata = LeRobotDatasetMetadata(REPO_ID)
    features = dataset_to_policy_features(dataset_metadata.features)

    output_features = {k: ft for k, ft in features.items() if ft.type is FeatureType.ACTION}
    input_features = {k: ft for k, ft in features.items() if k not in output_features}

    # --- choose cameras to keep (adjust as needed)
    cameras_to_keep = {
        "observation.images.top_camera",
        "observation.images.right_camera",
        "observation.images.front_camera",
        "observation.images.front_camera_depth",
        "observation.state",
    }
    input_features = {k: ft for k, ft in input_features.items() if k in cameras_to_keep}

    # --- configure diffusion policy
    cfg = DiffusionConfig(
        input_features=input_features,
        output_features=output_features,
        n_obs_steps=3,
        horizon=16,
        n_action_steps=8,
        drop_n_last_frames=7,
        num_train_timesteps=1000,
        num_inference_steps=30,
        crop_shape=(180, 240),
        use_separate_rgb_encoder_per_camera=True,
        crop_is_random=False,
        spatial_softmax_num_keypoints=48,
    )

    policy = DiffusionPolicy(cfg, dataset_stats=dataset_metadata.stats)
    policy.train()
    policy.to(device)
    preprocessor, postprocessor = make_pre_post_processors(cfg, dataset_stats=dataset_metadata.stats)

    # --- delta timestamps (one entry per image/state key)
    delta_timestamps = {}
    for key in input_features.keys():
        if "image" in key or "state" in key:
            delta_timestamps[key] = [i / dataset_metadata.fps for i in cfg.observation_delta_indices]
    # actions (same as before)
    delta_timestamps["action"] = [i / dataset_metadata.fps for i in cfg.action_delta_indices]

    # --- streaming dataset (doesn't load all data into RAM)
    # tolerance_s controls allowed timestamp tolerance; adjust shuffle_buffer_size if desired
    dataset = StreamingLeRobotDataset(REPO_ID, delta_timestamps=delta_timestamps, tolerance_s=1e-3)

    # --- optimizer / dataloader tuned for streaming
    optimizer = torch.optim.Adam(policy.parameters(), lr=1e-4)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        num_workers=8,                # increase for IO parallelism; tune for your machine
        batch_size=16,
        pin_memory=device.type != "cpu",
        drop_last=True,
        prefetch_factor=2,            # batches to prefetch per worker
        persistent_workers=True,      # keep workers alive for long runs
        shuffle=False,                # streaming dataset typically handles shuffle internally
    )

    # -------------------------------------------------
    # Training loop with checkpoints
    # -------------------------------------------------
    step = 0
    done = False
    while not done:
        for batch in dataloader:
            # Preprocess (converts raw frames -> tensors, normalization, stacking, etc.)
            batch = preprocessor(batch)
            batch = {k: (v.to(device) if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}

            loss, _ = policy.forward(batch)
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()

            if step % log_freq == 0:
                print(f"step: {step} | loss: {loss.item():.4f}")

            # Periodic checkpointing (by training step)
            if step > 0 and step % CHECKPOINT_FREQ == 0:
                ckpt_dir = checkpoints_dir / f"step_{step:06d}"
                ckpt_dir.mkdir(parents=True, exist_ok=True)
                policy.save_pretrained(ckpt_dir)
                preprocessor.save_pretrained(ckpt_dir)
                postprocessor.save_pretrained(ckpt_dir)
                print(f"[CHECKPOINT] Saved at step {step} -> {ckpt_dir}")

            step += 1
            if step >= TRAINING_STEPS:
                done = True
                break

    # -------------------------------------------------
    # Final save
    # -------------------------------------------------
    policy.save_pretrained(final_dir)
    preprocessor.save_pretrained(final_dir)
    postprocessor.save_pretrained(final_dir)

    print(f"[DONE] Final model saved to {final_dir}")


if __name__ == "__main__":
    main()
