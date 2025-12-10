"""
GPS overlay rendering for video frames.
Efficient text rendering with semi-transparent backgrounds.
"""

import cv2
import numpy as np
from typing import Tuple, Optional

from ..core.gps_trajectory import GPSPoint
from ..core.config import OverlayConfig
from ..utils.logger import get_logger


class GPSOverlay:
    """
    Renders GPS information overlay on video frames.
    
    Features:
    - Configurable position and styling
    - Semi-transparent background
    - Dynamic text sizing
    - Optimized rendering
    """
    
    def __init__(self, config: OverlayConfig):
        """
        Initialize GPS overlay renderer.
        
        Args:
            config: Overlay configuration
        """
        self.config = config
        self.logger = get_logger(__name__)
        
        # OpenCV font configuration
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = config.font_scale
        self.thickness = config.thickness
        self.line_type = cv2.LINE_AA  # Anti-aliased lines
        
        # Colors (BGR format)
        self.text_color = (255, 255, 255)  # White
        self.bg_color = (0, 0, 0)  # Black
        
        # Cache for text measurements
        self._text_size_cache = {}
    
    def render(
        self,
        frame: np.ndarray,
        gps_point: GPSPoint,
        speed_mps: Optional[float] = None,
        distance_m: Optional[float] = None,
        timestamp_ns: Optional[int] = None
    ) -> np.ndarray:
        """
        Render GPS overlay on frame.
        
        Complexity: O(1) - constant time text rendering
        
        Args:
            frame: Input video frame
            gps_point: GPS point to display
            speed_mps: Optional speed in m/s
            distance_m: Optional cumulative distance
            timestamp_ns: Optional timestamp
            
        Returns:
            Frame with overlay rendered
        """
        if not self.config.enabled:
            return frame
        
        # Prepare text lines
        lines = self._format_text_lines(
            gps_point, speed_mps, distance_m, timestamp_ns
        )
        
        if not lines:
            return frame
        
        # Calculate overlay dimensions
        box_x, box_y, box_w, box_h, line_height = self._calculate_box_dimensions(
            frame, lines
        )
        
        # Create overlay with transparency
        overlay = frame.copy()
        
        # Draw background rectangle
        cv2.rectangle(
            overlay,
            (box_x, box_y),
            (box_x + box_w, box_y + box_h),
            self.bg_color,
            -1  # Filled
        )
        
        # Blend overlay with original frame
        alpha = self.config.bg_alpha
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        
        # Render text lines
        padding = 15
        y_text = box_y + padding + line_height - 5
        
        for line in lines:
            cv2.putText(
                frame,
                line,
                (box_x + padding, y_text),
                self.font,
                self.font_scale,
                self.text_color,
                self.thickness,
                self.line_type
            )
            y_text += line_height
        
        return frame
    
    def _format_text_lines(
        self,
        gps_point: GPSPoint,
        speed_mps: Optional[float],
        distance_m: Optional[float],
        timestamp_ns: Optional[int]
    ) -> list:
        """
        Format text lines for overlay display.
        
        Args:
            gps_point: GPS point data
            speed_mps: Speed in m/s
            distance_m: Cumulative distance
            timestamp_ns: Timestamp
            
        Returns:
            List of formatted text strings
        """
        lines = []
        
        # GPS coordinates (always shown)
        lines.append(f"LAT: {gps_point.lat:.6f}")
        lines.append(f"LON: {gps_point.lon:.6f}")
        
        # Altitude
        if self.config.show_altitude:
            lines.append(f"ALT: {gps_point.alt:.1f}m")
        
        # Speed
        if self.config.show_speed and speed_mps is not None:
            speed_kmh = speed_mps * 3.6
            lines.append(f"SPD: {speed_kmh:.1f}km/h")
        
        # Distance
        if self.config.show_distance and distance_m is not None:
            if distance_m >= 1000:
                lines.append(f"DST: {distance_m / 1000:.2f}km")
            else:
                lines.append(f"DST: {distance_m:.1f}m")
        
        # Timestamp
        if self.config.show_timestamp and timestamp_ns is not None:
            timestamp_s = timestamp_ns / 1e9
            lines.append(f"T: {timestamp_s:.3f}s")
        
        return lines
    
    def _calculate_box_dimensions(
        self,
        frame: np.ndarray,
        lines: list
    ) -> Tuple[int, int, int, int, int]:
        """
        Calculate overlay box dimensions and position.
        
        Args:
            frame: Video frame
            lines: Text lines to display
            
        Returns:
            Tuple of (box_x, box_y, box_width, box_height, line_height)
        """
        h, w = frame.shape[:2]
        
        # Measure text sizes
        text_sizes = []
        for line in lines:
            # Check cache
            if line not in self._text_size_cache:
                size, _ = cv2.getTextSize(
                    line,
                    self.font,
                    self.font_scale,
                    self.thickness
                )
                self._text_size_cache[line] = size
            text_sizes.append(self._text_size_cache[line])
        
        # Calculate dimensions
        max_width = max(size[0] for size in text_sizes)
        line_height = max(size[1] for size in text_sizes) + 10
        
        padding = 15
        box_width = max_width + 2 * padding
        box_height = len(lines) * line_height + 2 * padding
        
        # Calculate position based on configuration
        margin = 10
        pos = self.config.position
        
        if pos == "top_left":
            box_x, box_y = margin, margin
        elif pos == "top_right":
            box_x = w - box_width - margin
            box_y = margin
        elif pos == "bottom_left":
            box_x = margin
            box_y = h - box_height - margin
        else:  # bottom_right
            box_x = w - box_width - margin
            box_y = h - box_height - margin
        
        return box_x, box_y, box_width, box_height, line_height
    
    def clear_cache(self):
        """Clear text size cache."""
        self._text_size_cache.clear()
