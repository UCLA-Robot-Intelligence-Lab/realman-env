#!/usr/bin/env python3
"""
LeRobot v3 dataset recorder for RealMan.

Records episodes in the LeRobotDataset v3 format (Parquet + MP4).
Each episode is a sequence of {observation, action, timestamp} frames.

Usage:
    recorder = LeRobotRecorder(
        repo_id="your-hf-username/dataset-name",
        root="./collected_data",
        fps=10,
        state_names=bridge.state_names,
        camera_names=["top", "left_wrist"],
    )
    recorder.start_episode()
    for frame in ...:
        recorder.add_frame(obs=state_vec, action=action_vec, images=img_dict)
    recorder.end_episode(task="pick and place")
    recorder.finalize()  # must call before push_to_hub
"""

import numpy as np
from typing import Optional

from constants import IMAGE_HEIGHT, IMAGE_WIDTH, DEFAULT_FPS


class LeRobotRecorder:
    """
    Wraps LeRobotDataset to record episodes in v3 format.

    Args:
        repo_id:      HuggingFace repo id (e.g. "user/my-dataset")
        root:         Local directory for dataset storage
        fps:          Recording frame rate
        state_names:  List of joint/state names (matches state vector length)
        camera_names: List of camera keys used in image dict
        robot_type:   Descriptive robot name stored in dataset metadata
    """

    def __init__(
        self,
        repo_id: str,
        root: str,
        fps: int = DEFAULT_FPS,
        state_names: Optional[list] = None,
        camera_names: Optional[list] = None,
        robot_type: str = "realman_dual_arm",
    ):
        self.repo_id = repo_id
        self.root = root
        self.fps = fps
        self.state_names = state_names or []
        self.camera_names = camera_names or []
        self.robot_type = robot_type

        self._dataset = None
        self._episode_frames = 0

        self._build_dataset()

    def _build_dataset(self):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        state_dim = len(self.state_names)
        features = {}

        if state_dim > 0:
            features["observation.state"] = {
                "dtype": "float32",
                "shape": (state_dim,),
                "names": self.state_names,
            }
            features["action"] = {
                "dtype": "float32",
                "shape": (state_dim,),
                "names": self.state_names,
            }

        for cam in self.camera_names:
            features[f"observation.images.{cam}"] = {
                "dtype": "video",
                "shape": (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
                "names": ["height", "width", "channel"],
            }

        self._dataset = LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            root=self.root,
            robot_type=self.robot_type,
            features=features,
        )
        print(f"Dataset ready: {self.repo_id} @ {self.root}")

    # ──────────────────────────────────────────────────────────────

    def start_episode(self):
        """Reset frame counter for a new episode."""
        self._episode_frames = 0

    def add_frame(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        images: dict,
    ):
        """
        Buffer one timestep.

        Args:
            obs:    State vector (float32, shape=(state_dim,))
            action: Action vector (float32, shape=(state_dim,))
            images: {camera_name: (H,W,C) uint8 BGR numpy array}
        """
        frame = {}

        if len(self.state_names) > 0:
            frame["observation.state"] = obs.astype(np.float32)
            frame["action"]            = action.astype(np.float32)

        for cam in self.camera_names:
            img = images.get(cam)
            if img is not None:
                frame[f"observation.images.{cam}"] = img  # (H,W,C) uint8

        self._dataset.add_frame(frame)
        self._episode_frames += 1

    def end_episode(self, task: str = ""):
        """Encode video and save episode to disk."""
        self._dataset.save_episode(task=task)
        n = self._episode_frames
        self._episode_frames = 0
        print(f"Episode saved: {n} frames, task='{task}'")
        return n

    def finalize(self):
        """
        Flush Parquet writers and write metadata footers.
        Must be called before push_to_hub() or the dataset will be corrupt.
        """
        self._dataset.finalize()
        print("Dataset finalized.")

    def push_to_hub(self):
        """Upload dataset to HuggingFace Hub."""
        self._dataset.push_to_hub()
        print(f"Pushed to Hub: {self.repo_id}")

    @property
    def num_episodes(self) -> int:
        return self._dataset.num_episodes if self._dataset else 0
