"""
Efficient ROS2 rosbag reader with data extraction capabilities.
Handles GPS and stereo camera data extraction with progress tracking.
"""

from pathlib import Path
from typing import Optional, Iterator, Tuple, List
import numpy as np
import cv2
from tqdm import tqdm

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from ..core.gps_trajectory import GPSTrajectory
from ..utils.time_sync import StereoSynchronizer
from ..utils.logger import get_logger


class RosbagReader:
    """
    High-level rosbag reader for GPS and camera data extraction.
    
    Features:
    - Automatic message type detection
    - Progress tracking with tqdm
    - Memory-efficient streaming
    - Compressed image support
    """
    
    def __init__(self, rosbag_path: str):
        """
        Initialize rosbag reader.
        
        Args:
            rosbag_path: Path to ROS2 rosbag directory
        """
        self.rosbag_path = Path(rosbag_path)
        if not self.rosbag_path.exists():
            raise FileNotFoundError(f"Rosbag not found: {rosbag_path}")
        
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        self.logger = get_logger(__name__)
        
        # Cache topic information
        self._topic_cache = {}
    
    def extract_gps_trajectory(
        self,
        gps_topic: str,
        gps_config: any  # GPSConfig
    ) -> Optional[GPSTrajectory]:
        """
        Extract complete GPS trajectory from rosbag.
        
        Complexity: O(n) where n is number of GPS messages
        
        Args:
            gps_topic: GPS topic name
            gps_config: GPS configuration with filter parameters
            
        Returns:
            GPSTrajectory instance or None if extraction fails
        """
        self.logger.info(f"Extracting GPS trajectory from topic: {gps_topic}")
        
        # Only use speed/acceleration filters if outlier filtering is enabled
        trajectory = GPSTrajectory(
            max_speed_mps=gps_config.max_speed_mps if gps_config.outlier_filter_enabled else 1000.0,
            max_acceleration_mps2=gps_config.max_acceleration_mps2 if gps_config.outlier_filter_enabled else 1000.0,
            min_satellites=gps_config.min_satellites,
            enable_filtering=gps_config.outlier_filter_enabled
        )
        
        with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
            # Find GPS connection
            gps_connections = [c for c in reader.connections if c.topic == gps_topic]
            
            if not gps_connections:
                self.logger.error(f"GPS topic not found: {gps_topic}")
                return None
            
            # Count messages for progress bar
            msg_count = sum(1 for _ in reader.messages(connections=gps_connections))
            
            # Reset reader
            with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
                gps_connections = [c for c in reader.connections if c.topic == gps_topic]
                
                # Process messages with progress bar
                for connection, timestamp, rawdata in tqdm(
                    reader.messages(connections=gps_connections),
                    total=msg_count,
                    desc="Reading GPS",
                    unit="msg"
                ):
                    try:
                        msg = reader.deserialize(rawdata, connection.msgtype)
                        
                        # Extract GPS data (NavSatFix message)
                        lat = float(msg.latitude)
                        lon = float(msg.longitude)
                        alt = float(msg.altitude) if hasattr(msg, 'altitude') else 0.0
                        
                        # Get number of satellites if available
                        num_sats = 0
                        if hasattr(msg, 'status') and hasattr(msg.status, 'service'):
                            num_sats = getattr(msg.status, 'service', 0)
                        
                        # Add point (will be filtered if invalid)
                        trajectory.add_point(timestamp, lat, lon, alt, num_sats)
                        
                    except Exception as e:
                        self.logger.debug(f"Error parsing GPS message: {e}")
                        continue
        
        if len(trajectory) == 0:
            self.logger.error("No valid GPS points extracted")
            return None
        
        stats = trajectory.get_statistics()
        self.logger.info(
            f"GPS extraction complete: {stats['num_points']} points, "
            f"{stats['filtered_points']} filtered, "
            f"{stats['total_distance_m']:.1f}m total distance"
        )
        
        return trajectory
    
    def extract_stereo_frames(
        self,
        left_topic: str,
        right_topic: str,
        sync_tolerance_ns: int,
        time_range: Optional[Tuple[int, int]] = None
    ) -> Iterator[Tuple[int, np.ndarray, np.ndarray, int, int]]:
        """
        Extract synchronized stereo frame pairs from rosbag.
        
        Yields only frames that have a synchronized pair within tolerance.
        
        Complexity: O(n log m) where n is left frames, m is right frames
        
        Args:
            left_topic: Left camera topic
            right_topic: Right camera topic
            sync_tolerance_ns: Maximum time difference for sync
            time_range: Optional (start_ns, end_ns) to filter by time
            
        Yields:
            Tuple of (left_timestamp, left_frame, right_frame, right_timestamp, time_diff_ns)
        """
        self.logger.info("Extracting synchronized stereo frames")
        
        # Window size: keep 5 seconds of right frames in buffer
        # At 24fps, this is ~120 frames, enough for good sync without excessive memory
        window_duration_ns = 5_000_000_000  # 5 seconds
        
        # Build timestamp index for right frames first (lightweight - only stores raw data)
        self.logger.info("Indexing right camera timestamps...")
        right_timestamps = {}
        with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
            right_conns = [c for c in reader.connections if c.topic == right_topic]
            
            if not right_conns:
                self.logger.error(f"Right camera topic not found: {right_topic}")
                return
            
            for connection, timestamp, rawdata in tqdm(
                reader.messages(connections=right_conns),
                desc="Indexing right camera",
                unit="frame"
            ):
                # Filter by time range
                if time_range and (timestamp < time_range[0] or timestamp > time_range[1]):
                    continue
                right_timestamps[timestamp] = (rawdata, connection.msgtype)
        
        self.logger.info(f"Indexed {len(right_timestamps)} right frames")
        
        # Sort right timestamps for efficient windowing
        sorted_right_ts = sorted(right_timestamps.keys())
        
        # Second pass: Process left frames with sliding window of right frames
        self.logger.info("Processing left camera frames...")
        synced_count = 0
        dropped_count = 0
        
        with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
            left_conns = [c for c in reader.connections if c.topic == left_topic]
            
            if not left_conns:
                self.logger.error(f"Left camera topic not found: {left_topic}")
                return
            
            # Cache for current window of right frames
            right_frame_cache = {}
            window_start_idx = 0
            
            for connection, timestamp, rawdata in tqdm(
                reader.messages(connections=left_conns),
                desc="Syncing stereo pairs",
                unit="frame"
            ):
                # Filter by time range
                if time_range and (timestamp < time_range[0] or timestamp > time_range[1]):
                    continue
                
                try:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    left_frame = self._decode_image(msg)
                    
                    if left_frame is None:
                        continue
                    
                    # Update right frame window: load frames within window of current left timestamp
                    window_start = timestamp - window_duration_ns
                    window_end = timestamp + window_duration_ns
                    
                    # Remove old frames from cache
                    right_frame_cache = {ts: frame for ts, frame in right_frame_cache.items() 
                                        if ts >= window_start}
                    
                    # Add new frames to cache within window
                    while window_start_idx < len(sorted_right_ts):
                        right_ts = sorted_right_ts[window_start_idx]
                        
                        if right_ts > window_end:
                            break
                        
                        if right_ts >= window_start and right_ts not in right_frame_cache:
                            # Load and decode this right frame
                            try:
                                rawdata, msgtype = right_timestamps[right_ts]
                                right_msg = reader.deserialize(rawdata, msgtype)
                                right_frame = self._decode_image(right_msg)
                                if right_frame is not None:
                                    right_frame_cache[right_ts] = right_frame
                            except Exception as e:
                                self.logger.debug(f"Error decoding right frame: {e}")
                        
                        window_start_idx += 1
                    
                    # Find best matching right frame within tolerance
                    best_match = None
                    min_diff = float('inf')
                    
                    for right_ts, right_frame in right_frame_cache.items():
                        time_diff = abs(timestamp - right_ts)
                        if time_diff <= sync_tolerance_ns and time_diff < min_diff:
                            min_diff = time_diff
                            best_match = (right_ts, right_frame)
                    
                    if best_match is None:
                        dropped_count += 1
                        del left_frame
                        continue
                    
                    right_timestamp, right_frame = best_match
                    
                    synced_count += 1
                    yield timestamp, left_frame, right_frame, right_timestamp, min_diff
                    
                    # Free memory immediately after yield
                    del left_frame, right_frame
                    
                except Exception as e:
                    self.logger.debug(f"Error processing left frame: {e}")
                    continue
        
        self.logger.info(
            f"Stereo sync complete: {synced_count} pairs, {dropped_count} dropped"
        )
    
    def _decode_image(self, msg: any) -> Optional[np.ndarray]:
        """
        Decode compressed or raw image message.
        
        Args:
            msg: ROS image message (CompressedImage or Image)
            
        Returns:
            Decoded image as numpy array or None
        """
        try:
            # Check if compressed
            if hasattr(msg, 'format'):  # CompressedImage
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                return img
            
            # Raw image
            elif hasattr(msg, 'encoding'):
                # Convert raw data to numpy array based on encoding
                if msg.encoding == 'bgr8':
                    img = np.frombuffer(msg.data, dtype=np.uint8)
                    img = img.reshape((msg.height, msg.width, 3))
                    return img
                elif msg.encoding == 'rgb8':
                    img = np.frombuffer(msg.data, dtype=np.uint8)
                    img = img.reshape((msg.height, msg.width, 3))
                    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                else:
                    self.logger.warning(f"Unsupported encoding: {msg.encoding}")
                    return None
            
            return None
            
        except Exception as e:
            self.logger.debug(f"Image decode error: {e}")
            return None
    
    def get_topic_list(self) -> List[str]:
        """
        Get list of all topics in rosbag.
        
        Returns:
            List of topic names
        """
        with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
            return [c.topic for c in reader.connections]
    
    def get_rosbag_duration(self) -> float:
        """
        Get rosbag duration in seconds.
        
        Returns:
            Duration in seconds
        """
        with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
            return reader.duration * 1e-9
