#!/usr/bin/env python3
"""
Main Data Collection Script for RealMan Dual-Arm System
Integrates hardware bridge, data recorder, and meta recorder
"""

import argparse
import signal
import sys
import time
import rclpy
from typing import Optional

from hardware_bridge_clean import RealmanHardwareBridge
from data_recorder import DataRecorder
from meta_recorder import MetaRecorder
from constants import DEFAULT_DATA_DIR


class DataCollectionSystem:
    """Integrated data collection system"""
    
    def __init__(self, dataset_name: str = "realman_dataset", output_dir: str = DEFAULT_DATA_DIR):
        self.output_dir = output_dir
        
        # Initialize components
        self.bridge = RealmanHardwareBridge()
        self.data_recorder = DataRecorder(output_dir)
        self.meta_recorder = MetaRecorder(dataset_name, output_dir)
        
        self.current_episode_id = None
        self.running = False
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("🤖 Data Collection System Initialized")
    
    def _signal_handler(self, signum, frame):
        """Handle interrupt signals"""
        print(f"\n🛑 Received signal {signum}, shutting down...")
        self.stop()
    
    def wait_for_ready(self, timeout: float = 30.0) -> bool:
        """Wait for hardware bridge to be ready"""
        print("⏳ Waiting for hardware connection...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.bridge, timeout_sec=0.1)
            
            if self.bridge.is_ready() and self.bridge.hands_ready() and self.bridge.poses_ready():
                print("✅ All systems ready!")
                print(f"   Left arm: {self.bridge._received_l_arm}")
                print(f"   Right arm: {self.bridge._received_r_arm}")
                print(f"   Left hand: {self.bridge._received_l_hand}")
                print(f"   Right hand: {self.bridge._received_r_hand}")
                print(f"   Arm poses: {self.bridge._received_l_pose and self.bridge._received_r_pose}")
                return True
        
        print("❌ Timeout waiting for systems to be ready")
        return False
    
    def start_collection(self, task_name: Optional[str] = None, task_description: Optional[str] = None,
                        duration: Optional[float] = None, fps: int = 10):
        """Start data collection"""
        if self.running:
            print("⚠ Collection already running!")
            return
        
        # Start episode in meta recorder
        self.current_episode_id = self.meta_recorder.start_episode(task_name, task_description)
        
        # Start data recording
        self.data_recorder.start_recording(self.bridge, duration, fps)
        
        self.running = True
        print(f"🔴 Data collection started (Episode: {self.current_episode_id})")
    
    def monitor_collection(self):
        """Monitor ongoing data collection"""
        if not self.running:
            return
        
        try:
            while self.running:
                rclpy.spin_once(self.bridge, timeout_sec=0.1)
                
                # Update meta recorder with progress
                stats = self.data_recorder.get_stats()
                self.meta_recorder.update_episode_progress(
                    stats['frames_recorded'],
                    fps=10.0  # Could be calculated from actual frame rate
                )
                
                time.sleep(0.5)  # Monitor at 2Hz
                
        except KeyboardInterrupt:
            pass
    
    def stop_collection(self, success: Optional[bool] = None, notes: Optional[str] = None):
        """Stop data collection"""
        if not self.running:
            return
        
        print("🛑 Stopping data collection...")
        self.running = False
        
        # Stop data recorder
        self.data_recorder.stop_recording()
        
        # Get file list
        stats = self.data_recorder.get_stats()
        file_list = []  # Could generate actual file list from directory
        
        # End episode in meta recorder
        self.meta_recorder.end_episode(success, notes, file_list)
        
        # Print summary
        self.meta_recorder.print_dataset_info()
    
    def stop(self):
        """Stop the system"""
        if self.running:
            self.stop_collection()
        
        # Cleanup
        self.bridge.cleanup()
        self.bridge.destroy_node()
        rclpy.shutdown()
    
    def interactive_mode(self):
        """Interactive data collection mode"""
        print("\n🎮 Interactive Data Collection Mode")
        print("Commands: 'start', 'stop', 'status', 'info', 'quit'")
        
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'start':
                    task = input("Task name (optional): ").strip() or None
                    desc = input("Task description (optional): ").strip() or None
                    duration = input("Duration in seconds (optional): ").strip()
                    duration = float(duration) if duration else None
                    
                    self.start_collection(task, desc, duration)
                    self.monitor_collection()
                
                elif cmd == 'stop':
                    success = input("Success? (y/n): ").strip().lower() == 'y'
                    notes = input("Notes (optional): ").strip() or None
                    self.stop_collection(success, notes)
                
                elif cmd == 'status':
                    if self.running:
                        stats = self.data_recorder.get_stats()
                        print(f"🔴 Recording: {stats['frames_recorded']} frames")
                    else:
                        print("⏸ Not recording")
                
                elif cmd == 'info':
                    self.meta_recorder.print_dataset_info()
                
                elif cmd in ['quit', 'exit']:
                    break
                
                else:
                    print("Unknown command. Available: start, stop, status, info, quit")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        self.stop()


def main():
    parser = argparse.ArgumentParser(description='RealMan Dual-Arm Data Collection')
    parser.add_argument('--dataset', default='realman_dataset', help='Dataset name')
    parser.add_argument('--output', default=DEFAULT_DATA_DIR, help='Output directory')
    parser.add_argument('--task', help='Task name')
    parser.add_argument('--description', help='Task description')
    parser.add_argument('--duration', type=float, help='Duration in seconds')
    parser.add_argument('--fps', type=int, default=10, help='Recording FPS')
    parser.add_argument('--interactive', action='store_true', help='Interactive mode')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create data collection system
        system = DataCollectionSystem(args.dataset, args.output)
        
        # Wait for systems to be ready
        if not system.wait_for_ready():
            return 1
        
        if args.interactive:
            # Interactive mode
            system.interactive_mode()
        else:
            # Automated mode
            system.start_collection(args.task, args.description, args.duration, args.fps)
            system.monitor_collection()
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())