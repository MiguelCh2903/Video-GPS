"""
Camera rectification and undistortion using calibration parameters.
Optimized with pre-computed remap tables for O(1) per-pixel operations.
"""

import json
import numpy as np
import cv2
from pathlib import Path
from typing import Tuple, Dict, Any

from ..utils.logger import get_logger


class CameraRectifier:
    """
    Efficient camera image rectification using pre-computed maps.
    
    The rectification maps are computed once during initialization,
    making subsequent rectify() calls very fast O(width * height).
    """
    
    def __init__(
        self,
        calibration_path: str,
        camera_side: str = "left",
        quality: str = "high"
    ):
        """
        Initialize camera rectifier with calibration data.
        
        Args:
            calibration_path: Path to stereo calibration JSON file
            camera_side: "left" or "right" camera
            quality: Rectification quality ("low", "medium", "high")
                    - high: alpha=0, crops black pixels, best quality
                    - medium: alpha=0.5, balance between quality and FOV
                    - low: alpha=1, keeps all pixels, fastest
        """
        self.logger = get_logger(__name__)
        self.camera_side = camera_side
        self.quality = quality
        
        # Load calibration
        self.calibration = self._load_calibration(calibration_path)
        
        # Get camera-specific parameters
        if camera_side == "left":
            self.mtx = np.array(self.calibration['mtx_left'])
            self.dist = np.array(self.calibration['dist_left']).flatten()
        elif camera_side == "right":
            self.mtx = np.array(self.calibration['mtx_right'])
            self.dist = np.array(self.calibration['dist_right']).flatten()
        else:
            raise ValueError(f"Invalid camera_side: {camera_side}")
        
        self.img_size = tuple(self.calibration['image_size'])
        
        # Pre-compute rectification maps
        self._initialize_maps()
        
        self.logger.info(
            f"Initialized {camera_side} camera rectifier "
            f"(quality={quality}, size={self.img_size})"
        )
    
    def _load_calibration(self, calib_path: str) -> Dict[str, Any]:
        """Load calibration from JSON file."""
        path = Path(calib_path)
        if not path.exists():
            raise FileNotFoundError(f"Calibration file not found: {calib_path}")
        
        with open(path, 'r') as f:
            calib = json.load(f)
        
        # Validate required fields
        required_fields = [
            'mtx_left', 'dist_left', 'mtx_right', 'dist_right',
            'image_size', 'R', 'T'
        ]
        for field in required_fields:
            if field not in calib:
                raise ValueError(f"Missing calibration field: {field}")
        
        return calib
    
    def _initialize_maps(self):
        """
        Pre-compute undistortion and rectification maps.
        
        This is the expensive operation (O(width * height)),
        but only done once during initialization.
        """
        # Select alpha based on quality
        alpha_map = {
            'high': 0.0,    # Crop all invalid pixels
            'medium': 0.5,  # Balance
            'low': 1.0      # Keep all pixels
        }
        alpha = alpha_map.get(self.quality, 0.0)
        
        # Get optimal new camera matrix
        self.new_mtx, self.roi = cv2.getOptimalNewCameraMatrix(
            self.mtx,
            self.dist,
            self.img_size,
            alpha,
            self.img_size
        )
        
        # Pre-compute remap tables
        # These maps tell us where to sample from the distorted image
        # for each pixel in the undistorted image
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.mtx,
            self.dist,
            None,  # No additional rectification rotation
            self.new_mtx,
            self.img_size,
            cv2.CV_32FC1  # Float32 for sub-pixel accuracy
        )
        
        # Extract ROI (Region of Interest) coordinates
        self.x, self.y, self.w, self.h = self.roi
        
        self.logger.debug(
            f"Rectification maps computed: ROI=({self.x},{self.y},{self.w},{self.h})"
        )
    
    def rectify(self, img: np.ndarray) -> np.ndarray:
        """
        Rectify (undistort) image using pre-computed maps.
        
        This operation is very fast since maps are pre-computed.
        Complexity: O(width * height) with optimized OpenCV implementation
        
        Args:
            img: Input distorted image
            
        Returns:
            Rectified (undistorted) image
        """
        # Apply remap transformation
        # This is highly optimized in OpenCV using SIMD instructions
        undistorted = cv2.remap(
            img,
            self.map1,
            self.map2,
            cv2.INTER_LINEAR  # Linear interpolation for speed
        )
        
        # Crop to valid ROI if specified
        if self.w > 0 and self.h > 0:
            return undistorted[self.y:self.y + self.h, self.x:self.x + self.w]
        
        return undistorted
    
    def get_output_size(self) -> Tuple[int, int]:
        """
        Get output image size after rectification.
        
        Returns:
            Tuple of (width, height)
        """
        if self.w > 0 and self.h > 0:
            return (self.w, self.h)
        return self.img_size


class StereoRectifier:
    """
    Stereo camera pair rectifier.
    Manages rectification for both left and right cameras.
    """
    
    def __init__(
        self,
        calibration_path: str,
        quality: str = "high"
    ):
        """
        Initialize stereo rectifier.
        
        Args:
            calibration_path: Path to stereo calibration file
            quality: Rectification quality
        """
        self.left_rectifier = CameraRectifier(
            calibration_path,
            camera_side="left",
            quality=quality
        )
        self.right_rectifier = CameraRectifier(
            calibration_path,
            camera_side="right",
            quality=quality
        )
        
        # Load stereo-specific parameters
        with open(calibration_path, 'r') as f:
            calib = json.load(f)
        
        self.R = np.array(calib['R'])
        self.T = np.array(calib['T'])
        self.baseline_mm = calib.get('baseline_mm', 0.0)
        
        self.logger = get_logger(__name__)
        self.logger.info(f"Stereo rectifier initialized (baseline={self.baseline_mm:.2f}mm)")
    
    def rectify_stereo_pair(
        self,
        left_img: np.ndarray,
        right_img: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Rectify both stereo images.
        
        Args:
            left_img: Left camera image
            right_img: Right camera image
            
        Returns:
            Tuple of (rectified_left, rectified_right)
        """
        left_rect = self.left_rectifier.rectify(left_img)
        right_rect = self.right_rectifier.rectify(right_img)
        return left_rect, right_rect
    
    def rectify_left(self, img: np.ndarray) -> np.ndarray:
        """Rectify left camera image only."""
        return self.left_rectifier.rectify(img)
    
    def rectify_right(self, img: np.ndarray) -> np.ndarray:
        """Rectify right camera image only."""
        return self.right_rectifier.rectify(img)
