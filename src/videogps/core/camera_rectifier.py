"""
Camera rectification and undistortion using calibration parameters.
Optimized with pre-computed remap tables for O(1) per-pixel operations.
"""

import json
import numpy as np
import cv2
from pathlib import Path
from typing import Tuple, Dict, Any, Optional

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
        
        self.base_img_size = tuple(self.calibration['image_size'])
        self.active_img_size: Optional[Tuple[int, int]] = None
        self._map_cache: Dict[Tuple[int, int], Dict[str, Any]] = {}
        
        # Pre-compute rectification maps for calibration resolution
        self._initialize_maps(self.base_img_size)
        
        self.logger.info(
            f"Initialized {camera_side} camera rectifier "
            f"(quality={quality}, base_size={self.base_img_size})"
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
    
    def _initialize_maps(self, img_size: Tuple[int, int]):
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
        width, height = img_size
        base_width, base_height = self.base_img_size

        # Scale intrinsic matrix if runtime image size differs from calibration size.
        if (width, height) != (base_width, base_height):
            sx = width / base_width
            sy = height / base_height
            scaled_mtx = self.mtx.copy().astype(np.float64)
            scaled_mtx[0, 0] *= sx  # fx
            scaled_mtx[1, 1] *= sy  # fy
            scaled_mtx[0, 2] *= sx  # cx
            scaled_mtx[1, 2] *= sy  # cy
        else:
            scaled_mtx = self.mtx
        
        # Get optimal new camera matrix
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            scaled_mtx,
            self.dist,
            img_size,
            alpha,
            img_size
        )
        
        # Pre-compute remap tables
        # These maps tell us where to sample from the distorted image
        # for each pixel in the undistorted image
        map1, map2 = cv2.initUndistortRectifyMap(
            scaled_mtx,
            self.dist,
            None,  # No additional rectification rotation
            new_mtx,
            img_size,
            cv2.CV_32FC1  # Float32 for sub-pixel accuracy
        )
        
        # Extract ROI (Region of Interest) coordinates
        x, y, w, h = roi
        self._map_cache[img_size] = {
            "map1": map1,
            "map2": map2,
            "new_mtx": new_mtx,
            "roi": roi,
        }
        self._set_active_maps(img_size)
        
        self.logger.debug(
            f"Rectification maps computed for size={img_size}: ROI=({x},{y},{w},{h})"
        )

    def _set_active_maps(self, img_size: Tuple[int, int]) -> None:
        """Select cached maps for the requested image size."""
        maps = self._map_cache[img_size]
        self.active_img_size = img_size
        self.map1 = maps["map1"]
        self.map2 = maps["map2"]
        self.new_mtx = maps["new_mtx"]
        self.roi = maps["roi"]
        self.x, self.y, self.w, self.h = self.roi
    
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
        img_size = (img.shape[1], img.shape[0])  # (width, height)
        if img_size != self.active_img_size:
            if img_size not in self._map_cache:
                self.logger.info(
                    f"Adapting {self.camera_side} rectification maps from "
                    f"{self.base_img_size} to {img_size}"
                )
                self._initialize_maps(img_size)
            else:
                self._set_active_maps(img_size)

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
        if self.active_img_size is not None:
            return self.active_img_size
        return self.base_img_size


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
