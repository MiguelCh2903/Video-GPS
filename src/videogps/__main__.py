"""
Command-line interface for VideoGPS.
Provides user-friendly commands for video generation and configuration.
"""

import argparse
import sys
from pathlib import Path

from videogps.core.config import Config, create_default_config
from videogps.processors.video_generator import VideoGenerator
from videogps.utils.logger import setup_logger


def cmd_generate(args):
    """Generate video from rosbag with GPS overlay."""
    print("🎬 VideoGPS - Video Generation")
    print("=" * 60)
    
    # Validate rosbag path
    rosbag_path = Path(args.rosbag)
    if not rosbag_path.exists():
        print(f"❌ Rosbag path not found: {rosbag_path}")
        return 1
    
    # Load configuration
    config_path = Path(args.config)
    if not config_path.exists():
        print(f"❌ Configuration file not found: {config_path}")
        print("💡 Use 'python -m videogps init' to create a default config")
        return 1
    
    try:
        config = Config.from_yaml(str(config_path))
        
        # Set rosbag path from command line (required)
        config.rosbag_path = str(rosbag_path)
        
        # Override from command line if provided
        if args.output:
            config.video.output_dir = args.output
        if args.log_level:
            config.log_level = args.log_level
        
        # Generate video
        generator = VideoGenerator(config)
        success = generator.generate()
        
        return 0 if success else 1
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1


def cmd_init(args):
    """Initialize a new configuration file."""
    output_path = Path(args.output)
    
    if output_path.exists() and not args.force:
        print(f"❌ Configuration file already exists: {output_path}")
        print("💡 Use --force to overwrite")
        return 1
    
    try:
        # Create default config
        config = create_default_config(str(output_path))
        
        print(f"✅ Configuration file created: {output_path}")
        print()
        print("📝 Next steps:")
        print(f"   1. Edit {output_path} with your settings")
        print(f"   2. Run: python -m videogps generate --config {output_path}")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error creating configuration: {e}")
        return 1


def cmd_info(args):
    """Display rosbag information."""
    from videogps.processors.rosbag_reader import RosbagReader
    
    rosbag_path = Path(args.rosbag)
    if not rosbag_path.exists():
        print(f"❌ Rosbag not found: {rosbag_path}")
        return 1
    
    try:
        print(f"📦 Rosbag Information: {rosbag_path}")
        print("=" * 60)
        
        reader = RosbagReader(str(rosbag_path))
        
        # Get duration
        duration = reader.get_rosbag_duration()
        print(f"Duration: {duration:.2f} seconds")
        print()
        
        # Get topics
        topics = reader.get_topic_list()
        print(f"Topics ({len(topics)}):")
        for topic in sorted(topics):
            print(f"  • {topic}")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error reading rosbag: {e}")
        return 1


def cmd_cut_by_coords(args):
    """Cut video segment by GPS coordinates without re-encoding."""
    print("✂️  VideoGPS - Cut Video by GPS Coordinates")
    print("=" * 60)
    
    import subprocess
    import json
    from pathlib import Path
    from videogps.utils.geo_utils import haversine_distance
    
    video_path = Path(args.input)
    if not video_path.exists():
        print(f"❌ Video file not found: {video_path}")
        return 1
    
    # Check for GPS data file
    gps_json = video_path.with_suffix('.gps.json')
    if not gps_json.exists():
        print(f"❌ GPS data file not found: {gps_json}")
        print("💡 Make sure the video was generated with embed_gps_track: true")
        return 1
    
    try:
        # Load GPS data
        print("Loading GPS data...")
        with open(gps_json, 'r') as f:
            gps_data = json.load(f)
        
        points = gps_data['gps_data']
        if not points:
            print("❌ No GPS data found in file")
            return 1
        
        print(f"Loaded {len(points)} GPS points")
        print()
        
        # Find closest points to start and end coordinates
        tolerance_m = args.tolerance
        
        print(f"Start: {args.start_lat}, {args.start_lon}")
        print(f"End: {args.end_lat}, {args.end_lon}")
        print(f"Tolerance: {tolerance_m}m")
        print()
        
        # Find start point
        start_idx = None
        min_start_dist = float('inf')
        for i, point in enumerate(points):
            dist = haversine_distance(
                args.start_lat, args.start_lon,
                point['latitude'], point['longitude']
            )
            if dist < min_start_dist:
                min_start_dist = dist
                start_idx = i
        
        # Find end point
        end_idx = None
        min_end_dist = float('inf')
        for i, point in enumerate(points):
            dist = haversine_distance(
                args.end_lat, args.end_lon,
                point['latitude'], point['longitude']
            )
            if dist < min_end_dist:
                min_end_dist = dist
                end_idx = i
        
        # Validate points found
        if start_idx is None or end_idx is None:
            print("❌ Could not find GPS points")
            return 1
        
        if min_start_dist > tolerance_m:
            print(f"❌ Start point not found within {tolerance_m}m (closest: {min_start_dist:.1f}m)")
            return 1
        
        if min_end_dist > tolerance_m:
            print(f"❌ End point not found within {tolerance_m}m (closest: {min_end_dist:.1f}m)")
            return 1
        
        if start_idx >= end_idx:
            print("❌ Start point must be before end point in trajectory")
            return 1
        
        # Calculate relative timestamps (frame-based)
        fps = gps_data['metadata']['fps']
        start_frame = points[start_idx]['frame']
        end_frame = points[end_idx]['frame']
        
        # Convert frames to seconds for ffmpeg
        start_time_s = start_frame / fps
        duration_s = (end_frame - start_frame) / fps
        
        print(f"✅ Found start point at {min_start_dist:.1f}m (frame {start_frame})")
        print(f"✅ Found end point at {min_end_dist:.1f}m (frame {end_frame})")
        print(f"⏱️  Segment: {start_time_s:.2f}s to {start_time_s + duration_s:.2f}s ({duration_s:.2f}s duration)")
        print()
        
        # Generate output path
        output_path = Path(args.output) if args.output else video_path.with_name(
            f"{video_path.stem}_segment_A_B.mp4"
        )
        
        # Cut video using ffmpeg
        # Strategy: Use -ss BEFORE -i for fast input seeking
        # 1. Try stream copy first (instant, no quality loss)
        # 2. If copy fails (keyframe issues), use ultrafast re-encode
        print(f"Output: {output_path}")
        print("Cutting video segment...")
        
        # Method 1: Stream copy with accurate seek (FASTEST - instant)
        # -ss before -i = input seeking (fast)
        # -avoid_negative_ts make_zero = fix timestamp issues
        cmd_copy = [
            'ffmpeg',
            '-ss', str(start_time_s),  # Seek BEFORE input (fast)
            '-i', str(video_path),
            '-t', str(duration_s),
            '-c:v', 'copy',  # No re-encoding
            '-c:a', 'copy',
            '-avoid_negative_ts', 'make_zero',  # Fix timestamp issues
            '-movflags', '+faststart',
            '-y',
            str(output_path)
        ]
        
        # Method 2: Ultra-fast re-encode (FALLBACK - ~30sec for 1min video)
        cmd_ultrafast = [
            'ffmpeg',
            '-ss', str(start_time_s),  # Seek BEFORE input (fast)
            '-i', str(video_path),
            '-t', str(duration_s),
            '-c:v', 'libx264',
            '-preset', 'ultrafast',  # Fastest CPU encoding
            '-crf', '23',
            '-c:a', 'copy',
            '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart',
            '-y',
            str(output_path)
        ]
        
        # Try stream copy first (instant)
        success = False
        try:
            print("Attempting stream copy (no re-encoding)...")
            result = subprocess.run(
                cmd_copy,
                capture_output=True,
                text=True,
                check=True,
                timeout=60
            )
            print("✅ Used stream copy (instant, no quality loss)")
            success = True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print("⚠️  Stream copy had issues, using fast re-encode...")
            # Fallback to ultrafast encode
            try:
                result = subprocess.run(
                    cmd_ultrafast,
                    capture_output=True,
                    text=True,
                    check=True
                )
                print("✅ Used ultrafast CPU encoding")
                success = True
            except subprocess.CalledProcessError as e2:
                raise e2
        
        print(f"✅ Segment extracted successfully: {output_path}")
        print()
        print("💡 GPS data is preserved in the segment!")
        print(f"📍 Segment: ({args.start_lat}, {args.start_lon}) → ({args.end_lat}, {args.end_lon})")
        
        return 0
        
    except subprocess.CalledProcessError as e:
        print(f"❌ ffmpeg error: {e.stderr}")
        return 1
    except FileNotFoundError:
        print("❌ ffmpeg not found. Please install ffmpeg.")
        print("Download from: https://ffmpeg.org/download.html")
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


def cmd_cut_by_time(args):
    """Cut video segment by time without re-encoding."""
    print("✂️  VideoGPS - Cut Video by Time")
    print("=" * 60)
    
    import subprocess
    from pathlib import Path
    
    video_path = Path(args.input)
    if not video_path.exists():
        print(f"❌ Video file not found: {video_path}")
        return 1
    
    output_path = Path(args.output) if args.output else video_path.with_name(
        f"{video_path.stem}_segment_{args.start:.1f}_{args.end:.1f}.mp4"
    )
    
    try:
        print(f"Input: {video_path}")
        print(f"Start: {args.start}s")
        print(f"End: {args.end}s")
        print(f"Output: {output_path}")
        print()
        
        # ffmpeg command to cut segment with stream copy (no re-encoding)
        cmd = [
            'ffmpeg',
            '-ss', str(args.start),
            '-to', str(args.end),
            '-i', str(video_path),
            '-c', 'copy',
            '-map', '0',
            '-y',
            str(output_path)
        ]
        
        print("Running ffmpeg...")
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=True
        )
        
        print(f"✅ Segment extracted successfully: {output_path}")
        print()
        print("💡 GPS data is preserved in the segment!")
        return 0
        
    except subprocess.CalledProcessError as e:
        print(f"❌ ffmpeg error: {e.stderr}")
        return 1
    except FileNotFoundError:
        print("❌ ffmpeg not found. Please install ffmpeg.")
        print("Download from: https://ffmpeg.org/download.html")
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1


def cmd_extract_segment(args):
    """Extract trajectory segment by coordinates."""
    print("✂️  VideoGPS - Trajectory Segment Extraction")
    print("=" * 60)
    
    config_path = Path(args.config)
    if not config_path.exists():
        print(f"❌ Configuration file not found: {config_path}")
        return 1
    
    try:
        # Load configuration
        config = Config.from_yaml(str(config_path))
        
        # Override trajectory settings
        config.trajectory.start_lat = args.start_lat
        config.trajectory.start_lon = args.start_lon
        config.trajectory.end_lat = args.end_lat
        config.trajectory.end_lon = args.end_lon
        config.trajectory.tolerance_m = args.tolerance
        
        # Generate video with segment
        generator = VideoGenerator(config)
        success = generator.generate()
        
        return 0 if success else 1
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="videogps",
        description="Generate GPS-overlayed videos from ROS2 rosbags",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Initialize configuration
  python -m videogps init
  
  # Generate video (rosbag path required)
  python -m videogps generate rosbags/my_rosbag --config config.yaml
  
  # View rosbag info
  python -m videogps info rosbags/my_rosbag
  
  # Cut video segment by GPS coordinates
  python -m videogps cut video.mp4 \\
      --start-lat -12.021 --start-lon -77.054 \\
      --end-lat -12.023 --end-lon -77.055
        """
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Available commands")
    
    # Generate command
    parser_gen = subparsers.add_parser(
        "generate",
        help="Generate GPS-overlayed video"
    )
    parser_gen.add_argument(
        "rosbag",
        help="Path to ROS2 rosbag directory"
    )
    parser_gen.add_argument(
        "--config", "-c",
        default="config.yaml",
        help="Configuration file path (default: config.yaml)"
    )
    parser_gen.add_argument(
        "--output", "-o",
        help="Override output directory from config"
    )
    parser_gen.add_argument(
        "--log-level", "-l",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Override log level from config"
    )
    parser_gen.set_defaults(func=cmd_generate)
    
    # Init command
    parser_init = subparsers.add_parser(
        "init",
        help="Create default configuration file"
    )
    parser_init.add_argument(
        "--output", "-o",
        default="config.yaml",
        help="Output configuration file path (default: config.yaml)"
    )
    parser_init.add_argument(
        "--force", "-f",
        action="store_true",
        help="Overwrite existing configuration file"
    )
    parser_init.set_defaults(func=cmd_init)
    
    # Info command
    parser_info = subparsers.add_parser(
        "info",
        help="Display rosbag information"
    )
    parser_info.add_argument(
        "rosbag",
        help="Path to rosbag directory"
    )
    parser_info.set_defaults(func=cmd_info)
    
    # Extract segment command
    parser_extract = subparsers.add_parser(
        "extract-segment",
        help="Extract and process trajectory segment"
    )
    parser_extract.add_argument(
        "--config", "-c",
        default="config.yaml",
        help="Configuration file path"
    )
    parser_extract.add_argument(
        "--start-lat",
        type=float,
        required=True,
        help="Start latitude"
    )
    parser_extract.add_argument(
        "--start-lon",
        type=float,
        required=True,
        help="Start longitude"
    )
    parser_extract.add_argument(
        "--end-lat",
        type=float,
        required=True,
        help="End latitude"
    )
    parser_extract.add_argument(
        "--end-lon",
        type=float,
        required=True,
        help="End longitude"
    )
    parser_extract.add_argument(
        "--tolerance",
        type=float,
        default=50.0,
        help="Distance tolerance in meters (default: 50.0)"
    )
    parser_extract.set_defaults(func=cmd_extract_segment)
    
    # Cut segment by GPS coordinates (main method)
    parser_cut_coords = subparsers.add_parser(
        "cut",
        help="Cut video segment by GPS coordinates (fast, preserves GPS)"
    )
    parser_cut_coords.add_argument(
        "input",
        help="Input video file with GPS track"
    )
    parser_cut_coords.add_argument(
        "--start-lat",
        type=float,
        required=True,
        help="Start latitude"
    )
    parser_cut_coords.add_argument(
        "--start-lon",
        type=float,
        required=True,
        help="Start longitude"
    )
    parser_cut_coords.add_argument(
        "--end-lat",
        type=float,
        required=True,
        help="End latitude"
    )
    parser_cut_coords.add_argument(
        "--end-lon",
        type=float,
        required=True,
        help="End longitude"
    )
    parser_cut_coords.add_argument(
        "--tolerance",
        type=float,
        default=50.0,
        help="Distance tolerance in meters (default: 50.0)"
    )
    parser_cut_coords.add_argument(
        "--output", "-o",
        help="Output video path (optional)"
    )
    parser_cut_coords.set_defaults(func=cmd_cut_by_coords)
    
    # Cut segment by time (alternative fast method)
    parser_cut_time = subparsers.add_parser(
        "cut-time",
        help="Cut video segment by time (fast, preserves GPS)"
    )
    parser_cut_time.add_argument(
        "input",
        help="Input video file with GPS track"
    )
    parser_cut_time.add_argument(
        "--start", "-s",
        type=float,
        required=True,
        help="Start time in seconds"
    )
    parser_cut_time.add_argument(
        "--end", "-e",
        type=float,
        required=True,
        help="End time in seconds"
    )
    parser_cut_time.add_argument(
        "--output", "-o",
        help="Output video path (optional)"
    )
    parser_cut_time.set_defaults(func=cmd_cut_by_time)
    
    # Parse arguments
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    # Execute command
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
