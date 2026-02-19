"""
Configuration management for VideoGPS.
Handles loading, validation, and storage of all configuration parameters.
"""

import yaml
from dataclasses import dataclass, field, asdict
from typing import Optional, Dict, Any
from pathlib import Path


@dataclass
class CameraConfig:
    """Camera-related configuration."""
    topic_left: str = "/camera_left/image_raw/compressed"
    topic_right: Optional[str] = "/camera_right/image_raw/compressed"
    sync_tolerance_ns: int = 50_000_000  # 50ms tolerance for stereo sync
    rectify_enabled: bool = True
    undistort_quality: str = "high"  # low, medium, high
    max_width: int = 1920


@dataclass
class GPSConfig:
    """GPS-related configuration."""
    topic: str = "/swift/navsat_fix"
    interpolation_enabled: bool = True
    outlier_filter_enabled: bool = False
    max_speed_mps: Optional[float] = 50.0  # Maximum realistic speed (m/s) - only used if outlier_filter_enabled
    max_acceleration_mps2: Optional[float] = 10.0  # Maximum realistic acceleration - only used if outlier_filter_enabled
    min_satellites: int = 4  # Minimum satellites for valid fix


@dataclass
class OverlayConfig:
    """Video overlay configuration."""
    enabled: bool = False
    position: str = "top_left"  # top_left, top_right, bottom_left, bottom_right
    font_scale: float = 0.7
    thickness: int = 2
    bg_alpha: float = 0.6
    show_speed: bool = True
    show_altitude: bool = True
    show_distance: bool = True
    show_timestamp: bool = False


@dataclass
class VideoConfig:
    """Video output configuration."""
    output_fps: int = 24
    codec: str = "libx264"  # libx264 (H.264), libx265 (H.265/HEVC)
    crf: int = 23  # Constant Rate Factor (0-51, lower=better quality, 23=default, 28=good balance)
    preset: str = "medium"  # ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow
    output_dir: str = "output"
    embed_gps_track: bool = True  # Embed GPS data as subtitle track


@dataclass
class Config:
    """Main configuration class for VideoGPS."""
    
    # Paths
    rosbag_path: str = ""  # Set via CLI argument, not config file
    calibration_path: str = "stereo_calib.json"
    
    # Sub-configurations
    camera: CameraConfig = field(default_factory=CameraConfig)
    gps: GPSConfig = field(default_factory=GPSConfig)
    overlay: OverlayConfig = field(default_factory=OverlayConfig)
    video: VideoConfig = field(default_factory=VideoConfig)
    
    # Logging
    log_level: str = "INFO"  # DEBUG, INFO, WARNING, ERROR
    log_file: Optional[str] = None
    
    @classmethod
    def from_yaml(cls, yaml_path: str) -> 'Config':
        """
        Load configuration from YAML file.
        
        Args:
            yaml_path: Path to YAML configuration file
            
        Returns:
            Config instance
        """
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # Parse nested configurations
        camera_data = data.pop('camera', {})
        gps_data = data.pop('gps', {})
        overlay_data = data.pop('overlay', {})
        video_data = data.pop('video', {})
        
        return cls(
            camera=CameraConfig(**camera_data),
            gps=GPSConfig(**gps_data),
            overlay=OverlayConfig(**overlay_data),
            video=VideoConfig(**video_data),
            **data
        )
    
    def to_yaml(self, yaml_path: str):
        """
        Save configuration to YAML file.
        
        Args:
            yaml_path: Path to output YAML file
        """
        data = {
            'calibration_path': self.calibration_path,
            'log_level': self.log_level,
            'log_file': self.log_file,
            'camera': asdict(self.camera),
            'gps': asdict(self.gps),
            'overlay': asdict(self.overlay),
            'video': asdict(self.video),
        }
        
        with open(yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    
    def validate(self) -> bool:
        """
        Validate configuration parameters.
        
        Returns:
            True if configuration is valid
            
        Raises:
            ValueError: If configuration is invalid
        """
        # Note: rosbag_path is validated in CLI, not here
        
        if self.camera.rectify_enabled and not Path(self.calibration_path).exists():
            raise ValueError(
                f"Calibration file does not exist (required when rectify_enabled=true): "
                f"{self.calibration_path}"
            )

        if not self.camera.topic_left and not self.camera.topic_right:
            raise ValueError("At least one camera topic must be configured")

        if self.camera.sync_tolerance_ns <= 0:
            raise ValueError("camera sync_tolerance_ns must be positive")
        
        if self.video.output_fps <= 0:
            raise ValueError("output_fps must be positive")
        
        if not 0 <= self.overlay.bg_alpha <= 1:
            raise ValueError("overlay bg_alpha must be between 0 and 1")
        
        return True


def create_default_config(output_path: str = "config.yaml") -> Config:
    """
    Create and save a default configuration file.
    
    Args:
        output_path: Path to save configuration
        
    Returns:
        Default Config instance
    """
    config = Config(
        calibration_path="stereo_calib.json",
    )
    config.to_yaml(output_path)
    return config
