#!/usr/bin/env python3
"""
Data Recorder for RealMan Dual-Arm System
Synchronized recording of robot state and camera data
"""

import numpy as np
import cv2
import json
import os
import threading
import time
from datetime import datetime
from typing import List, Dict, Optional
from dataclasses import dataclass

from constants import DEFAULT_DATA_DIR, MAX_BUFFER_SIZE
from hardware_bridge_clean import RealmanHardwareBridge


@dataclass
class DataPoint:
    """Single data point containing robot state and images"""
    timestamp: float
    robot_state: np.ndarray
    full_state: Dict
    images: List[np.ndarray]
    frame_id: int


class DataRecorder:
    """Synchronized data recorder for robot state and camera data"""
    
    def __init__(self, output_dir: str = DEFAULT_DATA_DIR):
        self.output_dir = output_dir
        self.recording = False
        self.frame_count = 0
        self.data_buffer = []
        
        # Threading
        self.record_thread = None
        self.stop_event = threading.Event()
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        print(f"📁 Data recorder initialized (output: {output_dir})")
    
    def start_recording(self, bridge: RealmanHardwareBridge, duration: Optional[float] = None, fps: int = 10):
        """Start recording synchronized data
        
        Args:
            bridge: Hardware bridge instance
            duration: Recording duration in seconds (None for infinite)
            fps: Recording frames per second
        """
        if self.recording:
            print("⚠ Already recording!")
            return
        
        self.recording = True
        self.frame_count = 0
        self.data_buffer.clear()
        self.stop_event.clear()
        
        print(f"🔴 Starting recording at {fps} FPS")
        if duration:
            print(f"⏱ Duration: {duration}s")
        
        # Start recording thread
        self.record_thread = threading.Thread(
            target=self._record_loop,
            args=(bridge, duration, fps)
        )
        self.record_thread.daemon = True
        self.record_thread.start()
    
    def _record_loop(self, bridge: RealmanHardwareBridge, duration: Optional[float], fps: int):
        """Main recording loop"""
        start_time = time.time()
        frame_interval = 1.0 / fps
        
        while self.recording and not self.stop_event.is_set():
            loop_start = time.time()
            
            # Check duration limit
            if duration and (time.time() - start_time) >= duration:
                break
            
            try:
                # Get synchronized data
                robot_state = bridge.get_state()
                full_state = bridge.get_full_state()
                images = bridge.get_camera_data(wait_for_fresh=False)
                
                # Create data point
                data_point = DataPoint(
                    timestamp=time.time(),
                    robot_state=robot_state.copy(),
                    full_state=full_state.copy(),
                    images=[img.copy() for img in images],
                    frame_id=self.frame_count
                )
                
                # Add to buffer
                self.data_buffer.append(data_point)
                
                # Save immediately to avoid memory issues
                self._save_data_point(data_point)
                
                self.frame_count += 1
                
                # Progress update
                if self.frame_count % fps == 0:  # Every second
                    elapsed = time.time() - start_time
                    print(f"  📹 Recorded {self.frame_count} frames ({elapsed:.1f}s elapsed)")
                
            except Exception as e:
                print(f"⚠ Recording error: {e}")
            
            # Rate limiting
            elapsed = time.time() - loop_start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print(f"✅ Recording completed: {self.frame_count} frames")
        self.recording = False
    
    def _save_data_point(self, data_point: DataPoint):
        """Save a single data point to disk"""
        # Save images
        camera_names = ['top_camera', 'left_wrist', 'right_wrist']
        for i, (img, name) in enumerate(zip(data_point.images, camera_names)):
            if img is not None and img.size > 0:
                filename = f"{self.output_dir}/{name}_{data_point.frame_id:06d}.png"
                cv2.imwrite(filename, img)
        
        # Save robot state
        state_filename = f"{self.output_dir}/robot_state_{data_point.frame_id:06d}.json"
        state_data = {
            'timestamp': data_point.timestamp,
            'frame_id': data_point.frame_id,
            'robot_state': data_point.robot_state.tolist(),
            'left_arm': data_point.robot_state[:6].tolist(),
            'left_hand': data_point.robot_state[6:12].tolist(),
            'right_arm': data_point.robot_state[12:18].tolist(),
            'right_hand': data_point.robot_state[18:24].tolist(),
            'full_state': data_point.full_state
        }
        
        with open(state_filename, 'w') as f:
            json.dump(state_data, f, indent=2, default=str)
    
    def stop_recording(self):
        """Stop recording"""
        if not self.recording:
            return
        
        print("🛑 Stopping recording...")
        self.stop_event.set()
        self.recording = False
        
        if self.record_thread and self.record_thread.is_alive():
            self.record_thread.join(timeout=5.0)
        
        # Save summary
        self._save_summary()
    
    def _save_summary(self):
        """Save recording summary"""
        summary = {
            'total_frames': self.frame_count,
            'recording_time': max([dp.timestamp for dp in self.data_buffer]) - min([dp.timestamp for dp in self.data_buffer]) if self.data_buffer else 0,
            'output_directory': self.output_dir,
            'data_structure': {
                'images': ['top_camera_XXXXXX.png', 'left_wrist_XXXXXX.png', 'right_wrist_XXXXXX.png'],
                'states': ['robot_state_XXXXXX.json']
            },
            'robot_state_dim': 24,  # left_arm(6) + left_hand(6) + right_arm(6) + right_hand(6)
            'camera_resolution': '640x480',
            'recording_date': datetime.now().isoformat()
        }
        
        summary_filename = os.path.join(self.output_dir, 'recording_summary.json')
        with open(summary_filename, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"📊 Recording summary saved to {summary_filename}")
    
    def get_stats(self) -> Dict:
        """Get recording statistics"""
        return {
            'frames_recorded': self.frame_count,
            'recording': self.recording,
            'buffer_size': len(self.data_buffer),
            'output_dir': self.output_dir
        }


def main():
    """Test data recorder"""
    print("Testing Data Recorder...")
    
    # Initialize hardware bridge
    import rclpy
    rclpy.init()
    
    try:
        bridge = RealmanHardwareBridge()
        
        # Wait for bridge to be ready
        print("Waiting for hardware bridge...")
        for i in range(50):  # 5 seconds
            rclpy.spin_once(bridge, timeout_sec=0.1)
            if bridge.is_ready() and bridge.hands_ready():
                print("✓ Hardware bridge ready!")
                break
        
        # Create data recorder
        recorder = DataRecorder("test_recording")
        
        # Start recording for 10 seconds
        recorder.start_recording(bridge, duration=10, fps=5)
        
        # Spin while recording
        start_time = time.time()
        while time.time() - start_time < 12:  # Extra time for cleanup
            rclpy.spin_once(bridge, timeout_sec=0.1)
            if not recorder.recording:
                break
        
        # Get stats
        stats = recorder.get_stats()
        print(f"📊 Stats: {stats}")
        
    finally:
        try:
            recorder.stop_recording()
        except:
            pass
        bridge.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()