"""
Main video generator orchestrating all components.
Handles the complete pipeline from rosbag to GPS-overlayed video.
"""

import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import Optional
import bisect

from ..core.config import Config
from ..core.camera_rectifier import CameraRectifier
from ..core.gps_trajectory import GPSTrajectory
from ..processors.rosbag_reader import RosbagReader
from ..processors.gps_overlay import GPSOverlay
from ..processors.gps_writer import GPSTrackWriter
from ..utils.logger import setup_logger, get_logger
import subprocess
import tempfile
import shutil
import threading
import queue


class VideoGenerator:
    """
    Main video generator class.
    
    Orchestrates the complete pipeline:
    1. Extract GPS trajectory from rosbag
    2. Filter trajectory by segment (optional)
    3. Extract synchronized stereo frames
    4. Rectify images
    5. Overlay GPS data
    6. Write output video
    """
    
    def __init__(self, config: Config):
        """
        Initialize video generator.
        
        Args:
            config: Configuration object
        """
        self.config = config
        
        # Setup logging
        setup_logger(
            "videogps",
            level=config.log_level,
            log_file=config.log_file
        )
        self.logger = get_logger(__name__)
        
        # Validate configuration
        config.validate()
        
        # Initialize components
        self.reader = RosbagReader(config.rosbag_path)
        self.overlay_renderer = GPSOverlay(config.overlay)
        
        # Camera rectifier (initialized later if needed)
        self.rectifier: Optional[CameraRectifier] = None
        
        # Trajectory data
        self.trajectory: Optional[GPSTrajectory] = None
        
        self.logger.info("VideoGenerator initialized")
    
    def generate(self) -> bool:
        """
        Generate GPS-overlayed video from rosbag.
        
        Returns:
            True if generation succeeded
        """
        try:
            self.logger.info("=" * 60)
            self.logger.info("Starting video generation")
            self.logger.info("=" * 60)
            
            # Step 1: Extract GPS trajectory
            self.logger.info("Step 1/5: Extracting GPS trajectory")
            self.trajectory = self.reader.extract_gps_trajectory(
                self.config.gps.topic,
                self.config.gps
            )
            
            if self.trajectory is None or len(self.trajectory) == 0:
                self.logger.error("Failed to extract GPS trajectory")
                return False
            
            # Log trajectory statistics
            stats = self.trajectory.get_statistics()
            self.logger.info(
                f"Trajectory: {stats['num_points']} points, "
                f"{stats['duration_s']:.1f}s duration, "
                f"{stats['total_distance_m']:.1f}m distance"
            )
            
            # Step 2: Filter trajectory by segment (optional)
            time_range = None
            if self._should_filter_trajectory():
                self.logger.info("Step 2/5: Filtering trajectory by coordinates")
                time_range = self._filter_trajectory()
                if time_range is None:
                    self.logger.error("Trajectory filtering failed")
                    return False
            else:
                self.logger.info("Step 2/5: Using full trajectory (no filtering)")
                time_range = (
                    self.trajectory.points[0].timestamp,
                    self.trajectory.points[-1].timestamp
                )
            
            # Step 3: Initialize camera rectifier
            if self.config.camera.rectify_enabled:
                self.logger.info("Step 3/5: Initializing camera rectifier")
                self.rectifier = CameraRectifier(
                    self.config.calibration_path,
                    camera_side="left",
                    quality=self.config.camera.undistort_quality
                )
            else:
                self.logger.info("Step 3/5: Camera rectification disabled")
            
            # Step 4: Process video
            self.logger.info("Step 4/5: Processing video frames")
            output_path = self._process_video_frames(time_range)
            
            if output_path is None:
                self.logger.error("Video processing failed")
                return False
            
            # Step 5: Complete
            self.logger.info("Step 5/5: Finalization")
            self.logger.info("=" * 60)
            self.logger.info(f"Video generated successfully: {output_path}")
            self.logger.info("=" * 60)
            
            return True
            
        except Exception as e:
            self.logger.exception(f"Video generation failed: {e}")
            return False
    
    def _should_filter_trajectory(self) -> bool:
        """Check if trajectory filtering is requested."""
        # Trajectory filtering disabled
        return False
    
    def _filter_trajectory(self) -> Optional[tuple]:
        """
        Filter trajectory by start/end coordinates.
        
        Returns:
            Tuple of (start_timestamp, end_timestamp) or None
        """
        traj_cfg = self.config.trajectory
        
        result = self.trajectory.filter_by_segment(
            traj_cfg.start_lat,
            traj_cfg.start_lon,
            traj_cfg.end_lat,
            traj_cfg.end_lon,
            traj_cfg.tolerance_m
        )
        
        if result is None:
            self.logger.error(
                f"Could not find trajectory segment within {traj_cfg.tolerance_m}m tolerance"
            )
            return None
        
        start_idx, end_idx = result
        
        # Extract segment
        self.trajectory = self.trajectory.extract_segment(start_idx, end_idx)
        
        stats = self.trajectory.get_statistics()
        self.logger.info(
            f"Filtered segment: {stats['num_points']} points, "
            f"{stats['duration_s']:.1f}s, {stats['total_distance_m']:.1f}m"
        )
        
        return (
            self.trajectory.points[0].timestamp,
            self.trajectory.points[-1].timestamp
        )
    
    def _process_video_frames(
        self,
        time_range: tuple
    ) -> Optional[Path]:
        """
        Process video frames with optional GPS overlay and embedded GPS track.
        
        Args:
            time_range: (start_timestamp, end_timestamp)
            
        Returns:
            Output video path or None
        """
        # Create output directory
        output_dir = Path(self.config.video.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate output filename
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        final_output = output_dir / f"gps_video_{timestamp_str}.mp4"
        
        # Create temporary video file (will be muxed with GPS later if enabled)
        temp_video = None
        if self.config.video.embed_gps_track:
            temp_dir = Path(tempfile.mkdtemp())
            temp_video = temp_dir / "video_temp.mp4"
            output_path = temp_video
        else:
            output_path = final_output
        
        # Initialize GPS track writer if embedding is enabled
        gps_writer = None
        if self.config.video.embed_gps_track:
            gps_srt_path = temp_dir / "gps_track.srt"
            gps_writer = GPSTrackWriter(
                str(gps_srt_path),
                fps=self.config.video.output_fps
            )
        
        # Video writer (initialized on first frame)
        video_writer = None
        use_ffmpeg = False  # Track which encoder is being used
        frame_count = 0
        accumulated_distance = 0.0
        
        try:
            # Extract synchronized stereo frames
            frame_iterator = self.reader.extract_stereo_frames(
                self.config.camera.topic_left,
                self.config.camera.topic_right,
                self.config.camera.sync_tolerance_ns,
                time_range
            )
            
            for left_ts, left_frame, right_frame, right_ts, time_diff in frame_iterator:
                # Use left frame for video (stereo pair is validated)
                frame = left_frame
                
                # Rectify if enabled
                if self.rectifier is not None:
                    frame = self.rectifier.rectify(frame)
                
                # Resize if exceeds max width
                h, w = frame.shape[:2]
                if w > self.config.camera.max_width:
                    scale = self.config.camera.max_width / w
                    new_w = self.config.camera.max_width
                    new_h = int(h * scale)
                    # Ensure height is even for H.264 encoding (required by libx264)
                    if new_h % 2 != 0:
                        new_h -= 1
                    frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
                
                # Get GPS point (with interpolation if enabled)
                gps_point = self.trajectory.get_point_at_time(
                    left_ts,
                    interpolate=self.config.gps.interpolation_enabled
                )
                
                if gps_point is None:
                    continue
                
                # Add GPS data to track writer
                if gps_writer is not None:
                    gps_writer.add_gps_point(left_ts, gps_point, frame_count)
                
                # Calculate speed and accumulated distance (only if overlay enabled)
                speed = 0.0
                if self.config.overlay.enabled:
                    # Binary search for GPS index (optimized)
                    gps_idx = bisect.bisect_left(
                        self.trajectory._timestamps,
                        gps_point.timestamp
                    )
                    
                    # Adjust if not exact match
                    if gps_idx < len(self.trajectory._timestamps):
                        if abs(self.trajectory._timestamps[gps_idx] - gps_point.timestamp) > 1e6:
                            if gps_idx > 0:
                                gps_idx -= 1
                    elif gps_idx > 0:
                        gps_idx -= 1
                    
                    if 0 <= gps_idx < len(self.trajectory.points):
                        speed = self.trajectory.calculate_speed_at(gps_idx)
                        
                        # Update accumulated distance
                        if gps_idx > 0:
                            prev_pt = self.trajectory.points[gps_idx - 1]
                            from ..utils.geo_utils import haversine_distance
                            dist = haversine_distance(
                                prev_pt.lat, prev_pt.lon,
                                gps_point.lat, gps_point.lon
                            )
                            accumulated_distance += dist
                    
                    # Render GPS overlay on frame
                    frame = self.overlay_renderer.render(
                        frame,
                        gps_point,
                        speed_mps=speed,
                        distance_m=accumulated_distance,
                        timestamp_ns=left_ts
                    )
                
                # Initialize video writer on first frame
                if video_writer is None:
                    h, w = frame.shape[:2]
                    
                    # Try to use ffmpeg for better H.264 compression
                    use_ffmpeg = False
                    try:
                        # Check if ffmpeg is available
                        subprocess.run(
                            ['ffmpeg', '-version'],
                            capture_output=True,
                            check=True
                        )
                        use_ffmpeg = True
                    except (FileNotFoundError, subprocess.CalledProcessError):
                        self.logger.warning(
                            "ffmpeg not found - using OpenCV encoder. "
                            "For better compression, install ffmpeg: https://ffmpeg.org/download.html"
                        )
                    
                    if use_ffmpeg:
                        # Use ffmpeg pipe for better H.264 compression
                        ffmpeg_cmd = [
                            'ffmpeg',
                            '-y',  # Overwrite output file
                            '-f', 'rawvideo',
                            '-vcodec', 'rawvideo',
                            '-s', f'{w}x{h}',
                            '-pix_fmt', 'bgr24',
                            '-r', str(self.config.video.output_fps),
                            '-i', '-',  # Read from stdin
                            '-an',  # No audio
                            '-vcodec', self.config.video.codec,
                            '-crf', str(self.config.video.crf),
                            '-preset', self.config.video.preset,
                            '-pix_fmt', 'yuv420p',  # Compatibility with most players
                            str(output_path)
                        ]
                        
                        self.logger.debug(f"FFmpeg command: {' '.join(ffmpeg_cmd)}")
                        
                        # Create stderr queue for non-blocking reads
                        stderr_queue = queue.Queue()
                        
                        video_writer = subprocess.Popen(
                            ffmpeg_cmd,
                            stdin=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            stdout=subprocess.DEVNULL
                        )
                        
                        # Thread to read stderr without blocking
                        def read_stderr():
                            try:
                                for line in video_writer.stderr:
                                    stderr_queue.put(line)
                            except:
                                pass
                        
                        stderr_thread = threading.Thread(target=read_stderr, daemon=True)
                        stderr_thread.start()
                        
                        self.logger.info(f"Encoder: FFmpeg (H.264 CRF {self.config.video.crf})")
                    else:
                        # Fallback to OpenCV VideoWriter with MP4V codec (better Windows compatibility)
                        # MP4V works reliably on Windows without additional libraries
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MPEG-4 Part 2
                        video_writer = cv2.VideoWriter(
                            str(output_path),
                            fourcc,
                            self.config.video.output_fps,
                            (w, h),
                            isColor=True
                        )
                        
                        if not video_writer.isOpened():
                            self.logger.error("Failed to initialize video writer")
                            return None
                        
                        self.logger.info(f"Encoder: OpenCV (MP4V)")
                    
                    self.logger.info(f"Temporary video: {output_path}")
                    self.logger.info(f"Resolution: {w}x{h} @ {self.config.video.output_fps}fps")
                    self.logger.info(f"Overlay: {'enabled' if self.config.overlay.enabled else 'disabled'}")
                
                # Write frame
                if use_ffmpeg:
                    try:
                        video_writer.stdin.write(frame.tobytes())
                    except (BrokenPipeError, OSError) as e:
                        self.logger.error(f"FFmpeg pipe error: {e}")
                        # Check if process is still running
                        if video_writer.poll() is not None:
                            self.logger.error(f"FFmpeg exited with code: {video_writer.returncode}")
                            # Collect any stderr output
                            stderr_lines = []
                            while not stderr_queue.empty():
                                try:
                                    stderr_lines.append(stderr_queue.get_nowait().decode('utf-8', errors='ignore').strip())
                                except:
                                    break
                            if stderr_lines:
                                self.logger.error("FFmpeg stderr output:")
                                for line in stderr_lines[-20:]:  # Last 20 lines
                                    if line:
                                        self.logger.error(f"  {line}")
                        del frame  # Free memory
                        break
                else:
                    video_writer.write(frame)
                
                # Free frame memory immediately after writing
                del frame
                frame_count += 1
            
            # Finalize video
            if video_writer is not None:
                if use_ffmpeg:
                    try:
                        video_writer.stdin.close()
                    except:
                        pass
                    video_writer.wait()
                    
                    # Check for ffmpeg errors
                    if video_writer.returncode != 0:
                        self.logger.error(f"FFmpeg exited with error code: {video_writer.returncode}")
                        # Collect any remaining stderr output
                        stderr_lines = []
                        while not stderr_queue.empty():
                            try:
                                stderr_lines.append(stderr_queue.get_nowait().decode('utf-8', errors='ignore').strip())
                            except:
                                break
                        if stderr_lines:
                            self.logger.error("FFmpeg stderr output:")
                            for line in stderr_lines[-20:]:  # Last 20 lines
                                if line:
                                    self.logger.error(f"  {line}")
                        return None
                else:
                    video_writer.release()
                
                duration_s = frame_count / self.config.video.output_fps
                self.logger.info(
                    f"Video encoding complete: {frame_count} frames, {duration_s:.2f}s duration"
                )
                
                # Mux GPS track into MP4 if enabled
                if self.config.video.embed_gps_track and gps_writer is not None:
                    self.logger.info("Step 5/5: Embedding GPS track into MP4")
                    
                    # Write GPS track
                    gps_srt = gps_writer.write_srt()
                    gps_json = gps_writer.write_json()
                    
                    # Log GPS statistics
                    stats = gps_writer.get_statistics()
                    self.logger.info(
                        f"GPS track: {stats['total_points']} points, "
                        f"{stats['avg_interval_s']:.3f}s avg interval"
                    )
                    
                    # Mux video + GPS using ffmpeg
                    success = self._mux_gps_track(temp_video, gps_srt, final_output)
                    
                    if success:
                        # Copy GPS JSON to output directory for reference
                        final_json = final_output.with_suffix('.gps.json')
                        shutil.copy(gps_json, final_json)
                        self.logger.info(f"GPS data backup: {final_json}")
                        
                        # Cleanup temp files
                        shutil.rmtree(temp_dir)
                        
                        return final_output
                    else:
                        self.logger.error("GPS track muxing failed, returning video without GPS")
                        shutil.copy(temp_video, final_output)
                        shutil.rmtree(temp_dir)
                        return final_output
                else:
                    return final_output
            else:
                self.logger.error("No frames were processed")
                return None
                
        except Exception as e:
            self.logger.exception(f"Error processing frames: {e}")
            if video_writer is not None:
                try:
                    if use_ffmpeg:
                        video_writer.stdin.close()
                        video_writer.terminate()
                    else:
                        video_writer.release()
                except:
                    pass
            if temp_video and temp_video.parent.exists():
                shutil.rmtree(temp_video.parent)
            return None
    
    def _mux_gps_track(
        self,
        video_path: Path,
        gps_srt_path: Path,
        output_path: Path
    ) -> bool:
        """
        Mux video with GPS subtitle track using ffmpeg.
        
        Args:
            video_path: Path to video file
            gps_srt_path: Path to GPS SRT file
            output_path: Path to output MP4
            
        Returns:
            True if successful
        """
        try:
            # ffmpeg command to mux video + GPS subtitle track
            cmd = [
                'ffmpeg',
                '-i', str(video_path),
                '-i', str(gps_srt_path),
                '-c:v', 'copy',  # Copy video stream without re-encoding
                '-c:s', 'mov_text',  # Convert SRT to MP4 subtitle format
                '-metadata:s:s:0', 'language=eng',
                '-metadata:s:s:0', 'title=GPS Data',
                '-y',  # Overwrite output
                str(output_path)
            ]
            
            self.logger.info("Muxing GPS track with ffmpeg...")
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True
            )
            
            self.logger.info(f"GPS track successfully embedded in: {output_path}")
            return True
            
        except subprocess.CalledProcessError as e:
            self.logger.error(f"ffmpeg error: {e.stderr}")
            return False
        except FileNotFoundError:
            self.logger.error(
                "ffmpeg not found. Please install ffmpeg to embed GPS tracks.\n"
                "Download from: https://ffmpeg.org/download.html"
            )
            return False
        except Exception as e:
            self.logger.error(f"Error muxing GPS track: {e}")
            return False


def generate_video_from_config(config_path: str) -> bool:
    """
    Convenience function to generate video from config file.
    
    Args:
        config_path: Path to YAML configuration file
        
    Returns:
        True if generation succeeded
    """
    config = Config.from_yaml(config_path)
    generator = VideoGenerator(config)
    return generator.generate()
