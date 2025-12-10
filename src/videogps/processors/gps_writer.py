"""
GPS track writer for embedding GPS data into MP4 files.
Writes timed GPS metadata compatible with standard MP4 tools.
"""

import struct
import json
from pathlib import Path
from typing import List, Optional
from datetime import datetime

from ..core.gps_trajectory import GPSPoint
from ..utils.logger import get_logger


class GPSTrackWriter:
    """
    Writes GPS data as a subtitle track in MP4 format.
    
    The GPS data is encoded as JSON in SRT subtitle format,
    which is later muxed into the MP4 container alongside video.
    This allows instant extraction and segmentation without reprocessing.
    """
    
    def __init__(self, output_path: str, fps: float = 24.0):
        """
        Initialize GPS track writer.
        
        Args:
            output_path: Path to output SRT file (temporary)
            fps: Video frame rate for synchronization
        """
        self.output_path = Path(output_path)
        self.fps = fps
        self.logger = get_logger(__name__)
        self.entries: List[dict] = []
        
    def add_gps_point(
        self,
        timestamp_ns: int,
        gps_point: GPSPoint,
        frame_number: int
    ):
        """
        Add GPS data point for a specific frame.
        
        Args:
            timestamp_ns: Frame timestamp in nanoseconds
            gps_point: GPS point data
            frame_number: Video frame number
        """
        entry = {
            'frame': frame_number,
            'timestamp_ns': timestamp_ns,
            'timestamp_s': timestamp_ns / 1e9,
            'latitude': gps_point.lat,
            'longitude': gps_point.lon,
            'altitude': gps_point.alt,
            'satellites': gps_point.num_satellites,
            'timestamp_iso': datetime.fromtimestamp(
                timestamp_ns / 1e9
            ).isoformat()
        }
        self.entries.append(entry)
    
    def write_srt(self) -> Path:
        """
        Write GPS data as SRT subtitle file.
        
        Returns:
            Path to written SRT file
        """
        self.logger.info(f"Writing GPS track with {len(self.entries)} points")
        
        with open(self.output_path, 'w', encoding='utf-8') as f:
            for idx, entry in enumerate(self.entries, 1):
                # Calculate timecodes
                start_s = entry['timestamp_s']
                end_s = start_s + (1.0 / self.fps)  # Duration of one frame
                
                # Format SRT timecode: HH:MM:SS,mmm
                start_tc = self._format_timecode(start_s)
                end_tc = self._format_timecode(end_s)
                
                # Write SRT entry
                f.write(f"{idx}\n")
                f.write(f"{start_tc} --> {end_tc}\n")
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
                f.write("\n")
        
        self.logger.info(f"GPS track written to: {self.output_path}")
        return self.output_path
    
    def write_json(self, json_path: Optional[str] = None) -> Path:
        """
        Write GPS data as standalone JSON file for backup/inspection.
        
        Args:
            json_path: Optional custom output path
            
        Returns:
            Path to written JSON file
        """
        if json_path is None:
            json_path = self.output_path.with_suffix('.json')
        else:
            json_path = Path(json_path)
        
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump({
                'metadata': {
                    'format': 'VideoGPS GPS Track',
                    'version': '1.0',
                    'fps': self.fps,
                    'total_points': len(self.entries)
                },
                'gps_data': self.entries
            }, f, indent=2, ensure_ascii=False)
        
        self.logger.info(f"GPS JSON backup written to: {json_path}")
        return json_path
    
    @staticmethod
    def _format_timecode(seconds: float) -> str:
        """
        Format seconds as SRT timecode.
        
        Args:
            seconds: Time in seconds
            
        Returns:
            Formatted timecode: HH:MM:SS,mmm
        """
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        millis = int((seconds % 1) * 1000)
        
        return f"{hours:02d}:{minutes:02d}:{secs:02d},{millis:03d}"
    
    def get_statistics(self) -> dict:
        """
        Get statistics about GPS track.
        
        Returns:
            Dictionary with statistics
        """
        if not self.entries:
            return {
                'total_points': 0,
                'duration_s': 0.0,
                'avg_interval_s': 0.0
            }
        
        duration = (
            self.entries[-1]['timestamp_s'] - 
            self.entries[0]['timestamp_s']
        )
        
        return {
            'total_points': len(self.entries),
            'duration_s': duration,
            'avg_interval_s': duration / len(self.entries) if len(self.entries) > 1 else 0.0,
            'first_timestamp': self.entries[0]['timestamp_iso'],
            'last_timestamp': self.entries[-1]['timestamp_iso']
        }


class GPSExtractor:
    """
    Extracts GPS data from VideoGPS-generated MP4 files.
    Can extract full data or segmented by time/coordinates.
    """
    
    def __init__(self, video_path: str):
        """
        Initialize GPS extractor.
        
        Args:
            video_path: Path to MP4 file with embedded GPS
        """
        self.video_path = Path(video_path)
        self.logger = get_logger(__name__)
    
    def extract_gps_srt(self, output_path: Optional[str] = None) -> Path:
        """
        Extract GPS subtitle track from MP4.
        
        Args:
            output_path: Optional output path for SRT
            
        Returns:
            Path to extracted SRT file
        """
        if output_path is None:
            output_path = self.video_path.with_suffix('.gps.srt')
        
        # This will be implemented using ffmpeg in video_generator
        # For now, return the expected path
        return Path(output_path)
    
    def extract_segment(
        self,
        start_time: float,
        end_time: float,
        output_path: str
    ) -> Path:
        """
        Extract video segment with GPS data by time range.
        
        Args:
            start_time: Start time in seconds
            end_time: End time in seconds
            output_path: Output video path
            
        Returns:
            Path to segmented video with GPS
        """
        # This will be implemented using ffmpeg stream copy
        # No re-encoding required
        return Path(output_path)
