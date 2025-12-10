"""
Time synchronization utilities for stereo cameras and GPS data.
Implements efficient timestamp matching with configurable tolerance.
"""

import bisect
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass


@dataclass
class TimestampedData:
    """Generic timestamped data container."""
    timestamp: int  # nanoseconds
    data: any


class TimeSynchronizer:
    """
    Efficient time synchronization for multiple data streams.
    Uses binary search for O(log n) lookups.
    """
    
    def __init__(self, tolerance_ns: int = 50_000_000):
        """
        Initialize synchronizer.
        
        Args:
            tolerance_ns: Maximum time difference for synchronization (default 50ms)
        """
        self.tolerance_ns = tolerance_ns
        self._timestamps: List[int] = []
        self._data: Dict[int, any] = {}
    
    def add_data(self, timestamp: int, data: any):
        """
        Add timestamped data to synchronizer.
        Maintains sorted order for efficient searching.
        
        Complexity: O(n) for insertion, but can be optimized with batch insertion
        
        Args:
            timestamp: Data timestamp in nanoseconds
            data: Associated data
        """
        bisect.insort(self._timestamps, timestamp)
        self._data[timestamp] = data
    
    def find_closest(self, target_timestamp: int) -> Optional[Tuple[int, any]]:
        """
        Find closest data point to target timestamp within tolerance.
        
        Complexity: O(log n) using binary search
        
        Args:
            target_timestamp: Target timestamp to match
            
        Returns:
            Tuple of (matched_timestamp, data) or None if no match within tolerance
        """
        if not self._timestamps:
            return None
        
        # Binary search for insertion point
        idx = bisect.bisect_left(self._timestamps, target_timestamp)
        
        # Check neighbors
        candidates = []
        if idx < len(self._timestamps):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        
        # Find closest within tolerance
        best_idx = None
        min_diff = float('inf')
        
        for i in candidates:
            diff = abs(self._timestamps[i] - target_timestamp)
            if diff < min_diff and diff <= self.tolerance_ns:
                min_diff = diff
                best_idx = i
        
        if best_idx is not None:
            ts = self._timestamps[best_idx]
            return ts, self._data[ts]
        
        return None
    
    def find_synchronized_pair(
        self,
        timestamp1: int,
        synchronizer2: 'TimeSynchronizer'
    ) -> Optional[Tuple[any, any]]:
        """
        Find synchronized pair of data from two synchronizers.
        
        Args:
            timestamp1: Timestamp from first stream
            synchronizer2: Second synchronizer to match against
            
        Returns:
            Tuple of (data1, data2) or None if no synchronized pair found
        """
        if timestamp1 not in self._data:
            return None
        
        result = synchronizer2.find_closest(timestamp1)
        if result is None:
            return None
        
        _, data2 = result
        return self._data[timestamp1], data2
    
    def clear(self):
        """Clear all stored data."""
        self._timestamps.clear()
        self._data.clear()
    
    def __len__(self) -> int:
        """Return number of stored timestamps."""
        return len(self._timestamps)


class StereoSynchronizer:
    """
    Specialized synchronizer for stereo camera pairs.
    Ensures left and right images are temporally aligned.
    """
    
    def __init__(self, tolerance_ns: int = 50_000_000):
        """
        Initialize stereo synchronizer.
        
        Args:
            tolerance_ns: Maximum time difference between stereo pairs
        """
        self.left_sync = TimeSynchronizer(tolerance_ns)
        self.right_sync = TimeSynchronizer(tolerance_ns)
        self.tolerance_ns = tolerance_ns
    
    def add_left_frame(self, timestamp: int, frame: any):
        """Add left camera frame."""
        self.left_sync.add_data(timestamp, frame)
    
    def add_right_frame(self, timestamp: int, frame: any):
        """Add right camera frame."""
        self.right_sync.add_data(timestamp, frame)
    
    def get_synchronized_frames(
        self,
        left_timestamp: int
    ) -> Optional[Tuple[any, any, int, int]]:
        """
        Get synchronized stereo pair for a given left frame timestamp.
        
        Args:
            left_timestamp: Left camera timestamp
            
        Returns:
            Tuple of (left_frame, right_frame, left_ts, right_ts) or None
        """
        result = self.left_sync.find_synchronized_pair(
            left_timestamp,
            self.right_sync
        )
        
        if result is None:
            return None
        
        left_frame, right_frame = result
        right_result = self.right_sync.find_closest(left_timestamp)
        
        if right_result is None:
            return None
        
        right_ts, _ = right_result
        return left_frame, right_frame, left_timestamp, right_ts
    
    def clear(self):
        """Clear all synchronized data."""
        self.left_sync.clear()
        self.right_sync.clear()
