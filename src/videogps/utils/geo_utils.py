"""
Geographic utility functions for GPS calculations.
Optimized implementations of haversine distance, bearing, and other geo operations.
"""

import numpy as np
from typing import Tuple


# Earth radius in meters
EARTH_RADIUS_M = 6371000.0


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate great-circle distance between two GPS points using Haversine formula.
    
    Complexity: O(1)
    
    Args:
        lat1: Latitude of first point (degrees)
        lon1: Longitude of first point (degrees)
        lat2: Latitude of second point (degrees)
        lon2: Longitude of second point (degrees)
        
    Returns:
        Distance in meters
    """
    # Convert to radians
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    
    # Haversine formula
    a = np.sin(dphi / 2) ** 2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    
    return EARTH_RADIUS_M * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate initial bearing between two GPS points.
    
    Args:
        lat1: Latitude of first point (degrees)
        lon1: Longitude of first point (degrees)
        lat2: Latitude of second point (degrees)
        lon2: Longitude of second point (degrees)
        
    Returns:
        Bearing in degrees (0-360)
    """
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dlambda = np.radians(lon2 - lon1)
    
    x = np.sin(dlambda) * np.cos(phi2)
    y = np.cos(phi1) * np.sin(phi2) - np.sin(phi1) * np.cos(phi2) * np.cos(dlambda)
    
    bearing = np.degrees(np.arctan2(x, y))
    return (bearing + 360) % 360


def interpolate_gps(
    lat1: float, lon1: float, alt1: float, t1: int,
    lat2: float, lon2: float, alt2: float, t2: int,
    t_target: int
) -> Tuple[float, float, float]:
    """
    Linear interpolation between two GPS points at a target timestamp.
    
    Args:
        lat1, lon1, alt1: First GPS point coordinates
        t1: First point timestamp (ns)
        lat2, lon2, alt2: Second GPS point coordinates
        t2: Second point timestamp (ns)
        t_target: Target timestamp for interpolation (ns)
        
    Returns:
        Tuple of (interpolated_lat, interpolated_lon, interpolated_alt)
    """
    if t2 == t1:
        return lat1, lon1, alt1
    
    # Linear interpolation factor
    alpha = (t_target - t1) / (t2 - t1)
    alpha = max(0.0, min(1.0, alpha))  # Clamp to [0, 1]
    
    # Interpolate each coordinate
    lat = lat1 + alpha * (lat2 - lat1)
    lon = lon1 + alpha * (lon2 - lon1)
    alt = alt1 + alpha * (alt2 - alt1)
    
    return lat, lon, alt


def validate_gps_coordinates(lat: float, lon: float) -> bool:
    """
    Validate GPS coordinates are within valid ranges.
    
    Args:
        lat: Latitude
        lon: Longitude
        
    Returns:
        True if coordinates are valid
    """
    return -90 <= lat <= 90 and -180 <= lon <= 180
