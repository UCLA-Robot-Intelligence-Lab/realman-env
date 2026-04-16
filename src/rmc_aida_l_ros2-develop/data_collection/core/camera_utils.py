#!/usr/bin/env python3
"""
Camera utilities for RealMan data collection
Handles RealSense camera operations
"""

import pyrealsense2 as rs
import numpy as np
import threading
import time
from typing import Dict, Optional
from constants import IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS


class AsyncCameraReader:
    """Async camera reader for multiple RealSense cameras"""
    
    def __init__(self, timeout_ms=100, slow_speed_threshold=15.0):
        self.timeout_ms = timeout_ms
        self.slow_speed_threshold = slow_speed_threshold
        self.stop_event = threading.Event()
        
        # Camera configurations
        self.camera_configs = [
            ('camera_0', "027322070274"),
            ('camera_1', "216322071814")
        ]
        
        self.pipelines = {}
        self.latest_frames = {}
        self.frame_lock = threading.Lock()
        self.running = False
        
        # Default image
        self._default_image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8)
        
        self._start_cameras()
    
    def _start_cameras(self):
        """Start all camera pipelines"""
        self.running = True
        
        for camera_name, serial in self.camera_configs:
            try:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(serial)
                config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
                config.enable_stream(rs.stream.depth, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.z16, 30)
                
                profile = pipeline.start(config)
                self.pipelines[camera_name] = pipeline
                
                # Initialize with default image
                self.latest_frames[camera_name] = self._default_image.copy()
                
                print(f"✓ Started {camera_name} (Serial: {serial})")
                
            except Exception as e:
                print(f"✗ Failed to start {camera_name}: {e}")
        
        if self.pipelines:
            # Start frame capture thread
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            print(f"✅ {len(self.pipelines)} camera(s) running")
    
    def _capture_loop(self):
        """Continuous frame capture loop"""
        while self.running and not self.stop_event.is_set():
            start_time = time.time()
            
            for camera_name, pipeline in self.pipelines.items():
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=self.timeout_ms)
                    
                    # Get color frame
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        color_image = np.asanyarray(color_frame.get_data())
                        
                        with self.frame_lock:
                            self.latest_frames[camera_name] = color_image
                    
                except Exception as e:
                    # Use default image on error
                    with self.frame_lock:
                        self.latest_frames[camera_name] = self._default_image.copy()
            
            # Rate limiting
            elapsed = (time.time() - start_time) * 1000
            if elapsed < 33:  # ~30 FPS
                time.sleep((33 - elapsed) / 1000.0)
    
    def get_latest_frame(self, timeout_ms=500, wait_for_fresh=True):
        """Get latest frames from all cameras"""
        deadline = time.time() + timeout_ms / 1000.0
        
        while wait_for_fresh and time.time() < deadline:
            with self.frame_lock:
                if self.latest_frames:
                    return self.latest_frames.copy()
            time.sleep(0.01)
        
        with self.frame_lock:
            return self.latest_frames.copy()
    
    def get_camera_data(self):
        """Get camera data in expected format [top, left_wrist, right_wrist]"""
        frames = self.get_latest_frame()
        
        # Map camera names to expected positions
        # For now, use camera_0 as top, camera_1 as left_wrist (adjust as needed)
        return [
            frames.get('camera_0', self._default_image),  # top_camera
            frames.get('camera_1', self._default_image),  # left_wrist
            self._default_image  # right_wrist (placeholder)
        ]
    
    def stop(self):
        """Stop all cameras"""
        self.stop_event.set()
        self.running = False
        
        for pipeline in self.pipelines.values():
            try:
                pipeline.stop()
            except:
                pass
        
        self.pipelines.clear()
        print("📷 Cameras stopped")