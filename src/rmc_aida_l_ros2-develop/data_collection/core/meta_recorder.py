#!/usr/bin/env python3
"""
Meta Data Recorder for RealMan Dual-Arm System
Records episode metadata, task information, and dataset statistics
"""

import json
import os
import time
from datetime import datetime
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict

from constants import DEFAULT_DATA_DIR, META_FILENAME


@dataclass
class EpisodeMetadata:
    """Metadata for a single recording episode"""
    episode_id: str
    start_time: float
    end_time: float
    duration: float
    total_frames: int
    fps: float
    task_name: Optional[str] = None
    task_description: Optional[str] = None
    success: Optional[bool] = None
    notes: Optional[str] = None
    robot_initial_pose: Optional[Dict] = None
    robot_final_pose: Optional[Dict] = None
    camera_info: Optional[Dict] = None
    file_list: Optional[List[str]] = None


@dataclass
class DatasetMetadata:
    """Metadata for the entire dataset"""
    dataset_name: str
    total_episodes: int
    total_frames: int
    total_duration: float
    creation_date: str
    last_modified: str
    robot_config: Dict
    camera_config: Dict
    environment_info: Dict
    task_types: List[str]
    episodes: List[EpisodeMetadata]


class MetaRecorder:
    """Records and manages metadata for data collection sessions"""
    
    def __init__(self, dataset_name: str = "realman_dataset", output_dir: str = DEFAULT_DATA_DIR):
        self.dataset_name = dataset_name
        self.output_dir = output_dir
        self.meta_file = os.path.join(output_dir, META_FILENAME)
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Load existing metadata or create new
        self.metadata = self._load_metadata()
        
        print(f"📊 Meta recorder initialized (dataset: {dataset_name})")
    
    def _load_metadata(self) -> DatasetMetadata:
        """Load existing metadata file or create new"""
        if os.path.exists(self.meta_file):
            try:
                with open(self.meta_file, 'r') as f:
                    data = json.load(f)
                
                # Convert dict back to dataclasses
                episodes = [EpisodeMetadata(**ep) for ep in data.get('episodes', [])]
                
                return DatasetMetadata(
                    dataset_name=data.get('dataset_name', self.dataset_name),
                    total_episodes=data.get('total_episodes', 0),
                    total_frames=data.get('total_frames', 0),
                    total_duration=data.get('total_duration', 0.0),
                    creation_date=data.get('creation_date', datetime.now().isoformat()),
                    last_modified=datetime.now().isoformat(),
                    robot_config=data.get('robot_config', {}),
                    camera_config=data.get('camera_config', {}),
                    environment_info=data.get('environment_info', {}),
                    task_types=data.get('task_types', []),
                    episodes=episodes
                )
            except Exception as e:
                print(f"⚠ Error loading metadata: {e}. Creating new metadata.")
        
        # Create new metadata
        return self._create_default_metadata()
    
    def _create_default_metadata(self) -> DatasetMetadata:
        """Create default dataset metadata"""
        return DatasetMetadata(
            dataset_name=self.dataset_name,
            total_episodes=0,
            total_frames=0,
            total_duration=0.0,
            creation_date=datetime.now().isoformat(),
            last_modified=datetime.now().isoformat(),
            robot_config={
                "type": "RealMan Dual-Arm",
                "arms": ["left", "right"],
                "dof_per_arm": 6,
                "hand_type": "6-DOF Dexterous Hand",
                "hand_dof": 6,
                "total_robot_dof": 24
            },
            camera_config={
                "type": "Intel RealSense D435",
                "count": 2,
                "resolution": "640x480",
                "fps": 30,
                "streams": ["color", "depth"],
                "camera_serials": ["027322070274", "216322071814"]
            },
            environment_info={
                "ros_version": "Foxy",
                "setup": "Development",
                "workspace": "rmc_aida_l_ros2-develop"
            },
            task_types=[],
            episodes=[]
        )
    
    def start_episode(self, task_name: Optional[str] = None, task_description: Optional[str] = None) -> str:
        """Start a new episode recording
        
        Returns:
            episode_id: Unique identifier for this episode
        """
        episode_id = f"episode_{self.metadata.total_episodes:04d}_{int(time.time())}"
        
        # Create episode metadata
        episode = EpisodeMetadata(
            episode_id=episode_id,
            start_time=time.time(),
            end_time=0.0,
            duration=0.0,
            total_frames=0,
            fps=10.0,  # Default FPS
            task_name=task_name,
            task_description=task_description,
            file_list=[]
        )
        
        # Add to current episodes tracking
        self._current_episode = episode
        
        print(f"🎬 Started episode: {episode_id}")
        if task_name:
            print(f"   Task: {task_name}")
        
        return episode_id
    
    def update_episode_progress(self, frames_recorded: int, fps: float):
        """Update current episode progress"""
        if hasattr(self, '_current_episode'):
            self._current_episode.total_frames = frames_recorded
            self._current_episode.fps = fps
            self._current_episode.duration = time.time() - self._current_episode.start_time
    
    def end_episode(self, success: Optional[bool] = None, notes: Optional[str] = None, 
                   file_list: Optional[List[str]] = None):
        """End current episode recording"""
        if not hasattr(self, '_current_episode'):
            print("⚠ No episode to end!")
            return
        
        # Update episode
        self._current_episode.end_time = time.time()
        self._current_episode.duration = self._current_episode.end_time - self._current_episode.start_time
        self._current_episode.success = success
        self._current_episode.notes = notes
        self._current_episode.file_list = file_list or []
        
        # Add to metadata
        self.metadata.episodes.append(self._current_episode)
        self.metadata.total_episodes += 1
        self.metadata.total_frames += self._current_episode.total_frames
        self.metadata.total_duration += self._current_episode.duration
        
        # Update task types
        if self._current_episode.task_name and self._current_episode.task_name not in self.metadata.task_types:
            self.metadata.task_types.append(self._current_episode.task_name)
        
        print(f"✅ Ended episode: {self._current_episode.episode_id}")
        print(f"   Duration: {self._current_episode.duration:.1f}s, Frames: {self._current_episode.total_frames}")
        
        # Clean up current episode
        delattr(self, '_current_episode')
        
        # Save metadata
        self.save_metadata()
    
    def save_metadata(self):
        """Save metadata to file"""
        self.metadata.last_modified = datetime.now().isoformat()
        
        # Convert dataclasses to dict for JSON serialization
        metadata_dict = {
            'dataset_name': self.metadata.dataset_name,
            'total_episodes': self.metadata.total_episodes,
            'total_frames': self.metadata.total_frames,
            'total_duration': self.metadata.total_duration,
            'creation_date': self.metadata.creation_date,
            'last_modified': self.metadata.last_modified,
            'robot_config': self.metadata.robot_config,
            'camera_config': self.metadata.camera_config,
            'environment_info': self.metadata.environment_info,
            'task_types': self.metadata.task_types,
            'episodes': [asdict(ep) for ep in self.metadata.episodes]
        }
        
        with open(self.meta_file, 'w') as f:
            json.dump(metadata_dict, f, indent=2)
        
        print(f"💾 Metadata saved to {self.meta_file}")
    
    def get_summary(self) -> Dict[str, Any]:
        """Get dataset summary"""
        return {
            'dataset_name': self.metadata.dataset_name,
            'total_episodes': self.metadata.total_episodes,
            'total_frames': self.metadata.total_frames,
            'total_duration_hours': self.metadata.total_duration / 3600,
            'average_episode_duration': self.metadata.total_duration / max(1, self.metadata.total_episodes),
            'average_fps': self.metadata.total_frames / max(1, self.metadata.total_duration),
            'task_types': self.metadata.task_types,
            'file_size_mb': self._get_dataset_size()
        }
    
    def _get_dataset_size(self) -> float:
        """Get total dataset size in MB"""
        total_size = 0
        try:
            for dirpath, dirnames, filenames in os.walk(self.output_dir):
                for filename in filenames:
                    filepath = os.path.join(dirpath, filename)
                    total_size += os.path.getsize(filepath)
        except Exception:
            pass
        
        return total_size / (1024 * 1024)  # Convert to MB
    
    def list_episodes(self, task_name: Optional[str] = None) -> List[EpisodeMetadata]:
        """List all episodes, optionally filtered by task name"""
        episodes = self.metadata.episodes
        if task_name:
            episodes = [ep for ep in episodes if ep.task_name == task_name]
        return episodes
    
    def get_episode(self, episode_id: str) -> Optional[EpisodeMetadata]:
        """Get specific episode by ID"""
        for episode in self.metadata.episodes:
            if episode.episode_id == episode_id:
                return episode
        return None
    
    def print_dataset_info(self):
        """Print formatted dataset information"""
        summary = self.get_summary()
        
        print(f"\n📊 Dataset: {summary['dataset_name']}")
        print(f"📁 Location: {self.output_dir}")
        print(f"📈 Episodes: {summary['total_episodes']}")
        print(f"🖼 Frames: {summary['total_frames']:,}")
        print(f"⏱ Duration: {summary['total_duration_hours']:.2f} hours")
        print(f"🎬 Avg Episode: {summary['average_episode_duration']:.1f}s")
        print(f"📷 Avg FPS: {summary['average_fps']:.1f}")
        print(f"💾 Size: {summary['file_size_mb']:.1f} MB")
        
        if summary['task_types']:
            print(f"🎯 Tasks: {', '.join(summary['task_types'])}")
        
        print()


def main():
    """Test meta recorder"""
    print("Testing Meta Recorder...")
    
    # Create meta recorder
    meta_recorder = MetaRecorder("test_dataset")
    
    # Start episode
    episode_id = meta_recorder.start_episode(
        task_name="test_recording",
        task_description="Testing meta recording functionality"
    )
    
    # Simulate recording
    time.sleep(2)
    meta_recorder.update_episode_progress(frames_recorded=50, fps=25.0)
    
    time.sleep(1)
    meta_recorder.update_episode_progress(frames_recorded=75, fps=25.0)
    
    # End episode
    meta_recorder.end_episode(
        success=True,
        notes="Test recording completed successfully",
        file_list=["test_data_1.json", "test_image_1.png"]
    )
    
    # Print dataset info
    meta_recorder.print_dataset_info()


if __name__ == '__main__':
    main()