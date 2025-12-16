"""
GPS trajectory management with interpolation and outlier filtering.
Optimized data structures for efficient temporal queries.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Tuple
import bisect

from ..utils.geo_utils import haversine_distance, interpolate_gps, validate_gps_coordinates
from ..utils.logger import get_logger


@dataclass
class GPSPoint:
    """Single GPS measurement point."""
    timestamp: int  # nanoseconds
    lat: float
    lon: float
    alt: float = 0.0
    speed: Optional[float] = None  # m/s
    num_satellites: int = 0
    
    def is_valid(self) -> bool:
        """Check if GPS point has valid coordinates."""
        return validate_gps_coordinates(self.lat, self.lon)


class GPSTrajectory:
    """
    GPS trajectory with interpolation and outlier filtering.
    
    Features:
    - O(log n) point lookup using binary search
    - GPS interpolation for missing timestamps
    - Outlier detection based on speed and acceleration
    - Distance and speed calculations
    """
    
    def __init__(
        self,
        max_speed_mps: float = 50.0,
        max_acceleration_mps2: float = 10.0,
        min_satellites: int = 4,
        enable_filtering: bool = True
    ):
        """
        Initialize GPS trajectory.
        
        Args:
            max_speed_mps: Maximum realistic speed for outlier detection
            max_acceleration_mps2: Maximum realistic acceleration
            min_satellites: Minimum satellites for valid fix
            enable_filtering: Enable outlier filtering
        """
        self.points: List[GPSPoint] = []
        self._timestamps: List[int] = []  # Sorted timestamps for binary search
        self.max_speed = max_speed_mps
        self.max_acceleration = max_acceleration_mps2
        self.min_satellites = min_satellites
        self.enable_filtering = enable_filtering
        self.logger = get_logger(__name__)
        
        # Statistics
        self.filtered_count = 0
        self.total_distance_m = 0.0
        
        # Batch mode optimization
        self._batch_points: List[GPSPoint] = []
    
    def add_point(
        self,
        timestamp: int,
        lat: float,
        lon: float,
        alt: float = 0.0,
        num_satellites: int = 0
    ) -> bool:
        """
        Add GPS point with validation and outlier filtering.
        
        OPTIMIZED: Accumulates points in batch for O(n log n) bulk insert.
        Call finalize_batch() after all points are added.
        
        Args:
            timestamp: Point timestamp in nanoseconds
            lat: Latitude
            lon: Longitude
            alt: Altitude
            num_satellites: Number of satellites in fix
            
        Returns:
            True if point was added, False if filtered out
        """
        # Validate coordinates
        if not validate_gps_coordinates(lat, lon):
            self.logger.debug(f"Invalid GPS coordinates: lat={lat}, lon={lon}")
            return False
        
        # Check minimum satellites
        if num_satellites < self.min_satellites and num_satellites > 0:
            self.logger.debug(f"Insufficient satellites: {num_satellites}")
            self.filtered_count += 1
            return False
        
        point = GPSPoint(timestamp, lat, lon, alt, num_satellites=num_satellites)
        
        # Add to batch instead of inserting directly
        self._batch_points.append(point)
        return True
    
    def finalize_batch(self):
        """
        Process accumulated batch points efficiently.
        Sorts all points at once: O(n log n) instead of O(n²).
        """
        if not self._batch_points:
            return
        
        # Sort batch by timestamp
        self._batch_points.sort(key=lambda p: p.timestamp)
        
        # Filter outliers if enabled
        if self.enable_filtering:
            filtered_points = []
            for point in self._batch_points:
                if len(filtered_points) == 0 or self._validate_point_against(point, filtered_points[-1]):
                    filtered_points.append(point)
                else:
                    self.filtered_count += 1
            self._batch_points = filtered_points
        
        # Add to main trajectory
        self.points.extend(self._batch_points)
        self._timestamps.extend([p.timestamp for p in self._batch_points])
        
        # Calculate total distance
        for i in range(1, len(self.points)):
            dist = haversine_distance(
                self.points[i-1].lat, self.points[i-1].lon,
                self.points[i].lat, self.points[i].lon
            )
            self.total_distance_m += dist
        
        # Clear batch
        self._batch_points = []
    
    def _validate_point_against(self, point: GPSPoint, prev_point: GPSPoint) -> bool:
        """
        Validate point against a specific previous point.
        
        Args:
            point: GPS point to validate
            prev_point: Previous GPS point
            
        Returns:
            True if point passes validation
        """
        # Calculate distance and time
        dist = haversine_distance(
            prev_point.lat, prev_point.lon,
            point.lat, point.lon
        )
        time_diff_s = (point.timestamp - prev_point.timestamp) / 1e9
        
        if time_diff_s <= 0:
            return False
        
        # Calculate instantaneous speed
        speed = dist / time_diff_s
        
        # Check maximum speed
        if speed > self.max_speed:
            self.logger.debug(f"Speed outlier detected: {speed:.2f} m/s")
            return False
        
        # Check acceleration if we have speed history
        if prev_point.speed is not None:
            acceleration = abs(speed - prev_point.speed) / time_diff_s
            if acceleration > self.max_acceleration:
                self.logger.debug(f"Acceleration outlier: {acceleration:.2f} m/s²")
                return False
        
        # Store calculated speed
        point.speed = speed
        
        return True
    
    def _validate_point(self, point: GPSPoint) -> bool:
        """
        Validate point against previous trajectory using physics constraints.
        
        Args:
            point: GPS point to validate
            
        Returns:
            True if point passes validation
        """
        if not self.points:
            return True
        
        prev_point = self.points[-1]
        
        # Calculate distance and time
        dist = haversine_distance(
            prev_point.lat, prev_point.lon,
            point.lat, point.lon
        )
        time_diff_s = (point.timestamp - prev_point.timestamp) / 1e9
        
        if time_diff_s <= 0:
            return False
        
        # Calculate instantaneous speed
        speed = dist / time_diff_s
        
        # Check maximum speed
        if speed > self.max_speed:
            self.logger.debug(f"Speed outlier detected: {speed:.2f} m/s")
            return False
        
        # Check acceleration if we have speed history
        if prev_point.speed is not None:
            acceleration = abs(speed - prev_point.speed) / time_diff_s
            if acceleration > self.max_acceleration:
                self.logger.debug(f"Acceleration outlier: {acceleration:.2f} m/s²")
                return False
        
        # Store calculated speed
        point.speed = speed
        
        return True
    
    def get_point_at_time(
        self,
        timestamp: int,
        interpolate: bool = True
    ) -> Optional[GPSPoint]:
        """
        Get GPS point at specific timestamp with optional interpolation.
        
        Complexity: O(log n) using binary search
        
        Args:
            timestamp: Target timestamp
            interpolate: Enable linear interpolation between points
            
        Returns:
            GPS point or None
        """
        if not self.points:
            return None
        
        # Binary search for closest timestamp
        idx = bisect.bisect_left(self._timestamps, timestamp)
        
        # Exact match
        if idx < len(self._timestamps) and self._timestamps[idx] == timestamp:
            return self.points[idx]
        
        # Find closest point
        if not interpolate:
            if idx == 0:
                return self.points[0]
            elif idx >= len(self._timestamps):
                return self.points[-1]
            else:
                # Return closest of the two neighbors
                diff_left = timestamp - self._timestamps[idx - 1]
                diff_right = self._timestamps[idx] - timestamp
                return self.points[idx - 1] if diff_left < diff_right else self.points[idx]
        
        # Interpolation
        if idx == 0:
            return self.points[0]
        elif idx >= len(self._timestamps):
            return self.points[-1]
        
        # Interpolate between idx-1 and idx
        p1 = self.points[idx - 1]
        p2 = self.points[idx]
        
        lat, lon, alt = interpolate_gps(
            p1.lat, p1.lon, p1.alt, p1.timestamp,
            p2.lat, p2.lon, p2.alt, p2.timestamp,
            timestamp
        )
        
        return GPSPoint(timestamp, lat, lon, alt)
    
    def filter_by_segment(
        self,
        start_lat: float,
        start_lon: float,
        end_lat: float,
        end_lon: float,
        tolerance_m: float = 50.0
    ) -> Optional[Tuple[int, int]]:
        """
        Find trajectory segment indices matching start and end coordinates.
        
        Optimized with best match finding instead of first match.
        Complexity: O(n) - single pass through trajectory
        
        Args:
            start_lat, start_lon: Start point coordinates
            end_lat, end_lon: End point coordinates
            tolerance_m: Distance tolerance in meters
            
        Returns:
            Tuple of (start_index, end_index) or None if not found
        """
        start_idx = None
        end_idx = None
        min_start_dist = float('inf')
        min_end_dist = float('inf')
        
        # Find best matching start and end points in single pass
        for i, pt in enumerate(self.points):
            # Check start point
            if start_idx is None:
                dist = haversine_distance(pt.lat, pt.lon, start_lat, start_lon)
                if dist < min_start_dist:
                    min_start_dist = dist
                    if dist <= tolerance_m:
                        start_idx = i
            
            # Only search for end after finding start
            if start_idx is not None and i >= start_idx:
                dist = haversine_distance(pt.lat, pt.lon, end_lat, end_lon)
                if dist < min_end_dist:
                    min_end_dist = dist
                    if dist <= tolerance_m:
                        end_idx = i
        
        if start_idx is None or end_idx is None:
            return None
        
        return start_idx, end_idx
    
    def extract_segment(self, start_idx: int, end_idx: int) -> 'GPSTrajectory':
        """
        Extract trajectory segment as new trajectory.
        
        Args:
            start_idx: Start index
            end_idx: End index (inclusive)
            
        Returns:
            New GPSTrajectory with segment data
        """
        segment = GPSTrajectory(
            self.max_speed,
            self.max_acceleration,
            self.min_satellites,
            self.enable_filtering
        )
        
        for point in self.points[start_idx:end_idx + 1]:
            segment.points.append(point)
            segment._timestamps.append(point.timestamp)
        
        # Recalculate total distance for segment
        segment._recalculate_distance()
        
        return segment
    
    def _recalculate_distance(self):
        """Recalculate total distance for trajectory."""
        self.total_distance_m = 0.0
        for i in range(1, len(self.points)):
            dist = haversine_distance(
                self.points[i - 1].lat, self.points[i - 1].lon,
                self.points[i].lat, self.points[i].lon
            )
            self.total_distance_m += dist
    
    def calculate_speed_at(self, idx: int, window: int = 3) -> float:
        """
        Calculate speed at trajectory index using moving window.
        
        Optimized with smaller default window for better responsiveness.
        
        Args:
            idx: Point index
            window: Window size for averaging (default: 3)
            
        Returns:
            Speed in m/s
        """
        if idx < window or idx >= len(self.points) - window:
            # Fallback for edge cases
            if idx > 0 and idx < len(self.points):
                prev_pt = self.points[idx - 1]
                curr_pt = self.points[idx]
                dist = haversine_distance(
                    prev_pt.lat, prev_pt.lon,
                    curr_pt.lat, curr_pt.lon
                )
                time_diff_s = (curr_pt.timestamp - prev_pt.timestamp) / 1e9
                return dist / time_diff_s if time_diff_s > 0 else 0.0
            return 0.0
        
        prev_pt = self.points[idx - window]
        next_pt = self.points[idx + window]
        
        dist = haversine_distance(
            prev_pt.lat, prev_pt.lon,
            next_pt.lat, next_pt.lon
        )
        time_diff_s = (next_pt.timestamp - prev_pt.timestamp) / 1e9
        
        return dist / time_diff_s if time_diff_s > 0 else 0.0
    
    def get_statistics(self) -> dict:
        """
        Get trajectory statistics.
        
        Returns:
            Dictionary with trajectory statistics
        """
        if not self.points:
            return {}
        
        duration_s = (self.points[-1].timestamp - self.points[0].timestamp) / 1e9
        avg_speed = self.total_distance_m / duration_s if duration_s > 0 else 0.0
        
        return {
            'num_points': len(self.points),
            'filtered_points': self.filtered_count,
            'duration_s': duration_s,
            'total_distance_m': self.total_distance_m,
            'avg_speed_mps': avg_speed,
            'avg_speed_kmh': avg_speed * 3.6,
            'start_time': self.points[0].timestamp,
            'end_time': self.points[-1].timestamp,
        }
    
    def __len__(self) -> int:
        """Return number of points in trajectory."""
        return len(self.points)
    
    def __getitem__(self, idx: int) -> GPSPoint:
        """Get point by index."""
        return self.points[idx]
