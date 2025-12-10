"""
VideoGPS - ROS2 Rosbag GPS Video Generator
Modular framework for generating GPS-overlayed videos from ROS2 rosbags.
"""

__version__ = "2.0.0"
__author__ = "VideoGPS Team"

from .core.config import Config
from .processors.video_generator import VideoGenerator

__all__ = ["Config", "VideoGenerator", "__version__"]
