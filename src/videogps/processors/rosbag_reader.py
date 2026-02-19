"""
Efficient ROS2 rosbag reader with data extraction capabilities.
Handles GPS and stereo camera data extraction with progress tracking.
"""

from pathlib import Path
from typing import Optional, Iterator, Tuple, List, Dict, Any
import numpy as np
import cv2
from tqdm import tqdm

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from ..core.gps_trajectory import GPSTrajectory
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

    def get_camera_availability(
        self,
        left_topic: Optional[str],
        right_topic: Optional[str],
        sync_tolerance_ns: int = 50_000_000,
        max_messages_to_sample: int = 300
    ) -> Dict[str, Any]:
        """
        Check whether configured camera topics exist and contain usable data.

        Args:
            left_topic: Left camera topic or None/empty
            right_topic: Right camera topic or None/empty
            sync_tolerance_ns: Stereo sync tolerance in nanoseconds
            max_messages_to_sample: Max messages to inspect per topic

        Returns:
            Dict with availability/quality flags and diagnostics
        """
        topics = set(self.get_topic_list())

        def _probe_topic(topic: Optional[str]) -> Dict[str, Any]:
            stats: Dict[str, Any] = {
                "configured": bool(topic),
                "topic": topic,
                "exists": False,
                "message_count": 0,
                "sampled_messages": 0,
                "nonempty_messages": 0,
                "decoded_frames": 0,
                "first_timestamp_ns": None,
                "last_timestamp_ns": None,
                "decoded_timestamps_ns": [],
                "usable": False,
            }

            if not topic:
                return stats

            stats["exists"] = topic in topics
            if not stats["exists"]:
                return stats

            with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
                conns = [c for c in reader.connections if c.topic == topic]
                stats["message_count"] = sum(c.msgcount for c in conns)

                for connection, timestamp, rawdata in reader.messages(connections=conns):
                    if stats["sampled_messages"] >= max_messages_to_sample:
                        break

                    stats["sampled_messages"] += 1
                    if stats["first_timestamp_ns"] is None:
                        stats["first_timestamp_ns"] = timestamp
                    stats["last_timestamp_ns"] = timestamp

                    try:
                        msg = reader.deserialize(rawdata, connection.msgtype)

                        payload = getattr(msg, "data", None)
                        if payload is not None and len(payload) > 0:
                            stats["nonempty_messages"] += 1

                        frame = self._decode_image(msg)
                        if frame is not None:
                            stats["decoded_frames"] += 1
                            stats["decoded_timestamps_ns"].append(timestamp)
                    except Exception:
                        continue

            stats["usable"] = stats["decoded_frames"] > 0 and stats["nonempty_messages"] > 0
            return stats

        left_stats = _probe_topic(left_topic)
        right_stats = _probe_topic(right_topic)

        sample_sync_pairs = 0
        if left_stats["usable"] and right_stats["usable"]:
            left_ts = left_stats["decoded_timestamps_ns"]
            right_ts = right_stats["decoded_timestamps_ns"]
            i, j = 0, 0
            while i < len(left_ts) and j < len(right_ts):
                diff = left_ts[i] - right_ts[j]
                abs_diff = abs(diff)
                if abs_diff <= sync_tolerance_ns:
                    sample_sync_pairs += 1
                    i += 1
                    j += 1
                elif diff < 0:
                    i += 1
                else:
                    j += 1

        return {
            "left_available": left_stats["exists"],
            "right_available": right_stats["exists"],
            "left_usable": left_stats["usable"],
            "right_usable": right_stats["usable"],
            "sample_sync_pairs": sample_sync_pairs,
            "left_stats": left_stats,
            "right_stats": right_stats,
        }
    
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
            
            # Use metadata count to avoid an expensive full pre-scan pass.
            msg_count = sum(c.msgcount for c in gps_connections) or None

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
        
        # Finalize batch processing for efficient sorting
        trajectory.finalize_batch()
        
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

        # Use streaming approach with limited buffer.
        # Keep recent right frames in memory (max 100 frames ~ 4 seconds @ 24fps).
        max_buffer_size = 100

        # Process with two simultaneous readers and limited buffer.
        self.logger.info("Processing stereo frames with streaming...")
        synced_count = 0
        dropped_count = 0
        
        # Open left reader
        with AnyReader([self.rosbag_path], default_typestore=self.typestore) as left_reader:
            left_conns = [c for c in left_reader.connections if c.topic == left_topic]
            
            if not left_conns:
                self.logger.error(f"Left camera topic not found: {left_topic}")
                return
            
            # Open right reader for on-demand loading
            with AnyReader([self.rosbag_path], default_typestore=self.typestore) as right_reader:
                right_conns = [c for c in right_reader.connections if c.topic == right_topic]
                if not right_conns:
                    self.logger.error(f"Right camera topic not found: {right_topic}")
                    return
                
                # Build iterator for right frames
                right_iter = right_reader.messages(connections=right_conns)
                
                # Limited circular buffer for right frames
                from collections import deque
                right_buffer = deque(maxlen=max_buffer_size)
                right_exhausted = False

                # Pre-load initial buffer
                try:
                    for _ in range(max_buffer_size):
                        connection, r_ts, rawdata = next(right_iter)
                        if time_range and (r_ts < time_range[0] or r_ts > time_range[1]):
                            continue
                        msg = right_reader.deserialize(rawdata, connection.msgtype)
                        r_frame = self._decode_image(msg)
                        if r_frame is not None:
                            right_buffer.append((r_ts, r_frame))
                except StopIteration:
                    right_exhausted = True

                # Process left frames
                for connection, timestamp, rawdata in tqdm(
                    left_reader.messages(connections=left_conns),
                    desc="Syncing stereo pairs",
                    unit="frame"
                ):
                    # Filter by time range
                    if time_range and (timestamp < time_range[0] or timestamp > time_range[1]):
                        continue
                    
                    try:
                        msg = left_reader.deserialize(rawdata, connection.msgtype)
                        left_frame = self._decode_image(msg)
                        
                        if left_frame is None:
                            continue

                        # Load more right frames if needed (keep buffer ahead)
                        if not right_exhausted:
                            last_right_ts = right_buffer[-1][0] if right_buffer else float("-inf")
                            while last_right_ts < timestamp + sync_tolerance_ns:
                                try:
                                    connection, r_ts, rawdata = next(right_iter)
                                    if time_range and (r_ts < time_range[0] or r_ts > time_range[1]):
                                        continue
                                    msg = right_reader.deserialize(rawdata, connection.msgtype)
                                    r_frame = self._decode_image(msg)
                                    if r_frame is not None:
                                        right_buffer.append((r_ts, r_frame))
                                        last_right_ts = r_ts
                                except StopIteration:
                                    right_exhausted = True
                                    break

                        # Drop stale right frames that cannot match current left timestamp
                        min_valid_ts = timestamp - sync_tolerance_ns
                        while right_buffer and right_buffer[0][0] < min_valid_ts:
                            right_buffer.popleft()

                        # Find best match in buffer
                        best_match = None
                        min_diff = float('inf')

                        for r_ts, r_frame in right_buffer:
                            time_diff = abs(timestamp - r_ts)
                            if time_diff <= sync_tolerance_ns and time_diff < min_diff:
                                min_diff = time_diff
                                best_match = (r_ts, r_frame)
                        
                        if best_match is None:
                            dropped_count += 1
                            del left_frame
                            continue
                        
                        right_timestamp, right_frame = best_match

                        synced_count += 1
                        yield timestamp, left_frame, right_frame, right_timestamp, min_diff

                    except Exception as e:
                        self.logger.debug(f"Error processing left frame: {e}")
                        continue
        
        self.logger.info(
            f"Stereo sync complete: {synced_count} pairs, {dropped_count} dropped"
        )

    def extract_monocular_frames(
        self,
        topic: str,
        time_range: Optional[Tuple[int, int]] = None
    ) -> Iterator[Tuple[int, np.ndarray]]:
        """
        Extract frames from a single camera topic.

        Args:
            topic: Camera topic name
            time_range: Optional (start_ns, end_ns) to filter by time

        Yields:
            Tuple of (timestamp, frame)
        """
        self.logger.info(f"Extracting monocular frames from topic: {topic}")

        with AnyReader([self.rosbag_path], default_typestore=self.typestore) as reader:
            conns = [c for c in reader.connections if c.topic == topic]

            if not conns:
                self.logger.error(f"Camera topic not found: {topic}")
                return

            for connection, timestamp, rawdata in tqdm(
                reader.messages(connections=conns),
                desc="Reading monocular frames",
                unit="frame"
            ):
                if time_range and (timestamp < time_range[0] or timestamp > time_range[1]):
                    continue

                try:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    frame = self._decode_image(msg)
                    if frame is not None:
                        yield timestamp, frame
                except Exception as e:
                    self.logger.debug(f"Error processing camera frame: {e}")
                    continue
    
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
