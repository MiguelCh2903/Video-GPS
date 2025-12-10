# VideoGPS

**Genera videos con GPS desde rosbags ROS2 con cámaras stereo sincronizadas**

## Instalación

```bash
pip install -e .
```

## Uso Rápido

### 1. Crear archivo de configuración

```bash
python -m videogps init
```

Esto crea `config.yaml` con la configuración por defecto.

### 2. Editar `config.yaml`

Configura:
- Topics de cámaras y GPS
- Ruta al archivo de calibración stereo
- Overlay GPS (deshabilitado por defecto)
- Configuración de video (FPS, codec, resolución)

### 3. Generar video

```bash
python -m videogps generate rosbags/tu_rosbag --config config.yaml
```

Esto genera:
- `output/gps_video_YYYYMMDD_HHMMSS.mp4` - Video con GPS embebido
- `output/gps_video_YYYYMMDD_HHMMSS.gps.json` - Datos GPS en JSON

### 4. Cortar segmentos por coordenadas GPS

```bash
python -m videogps cut output/gps_video_20251210_175328.mp4 \
    --start-lat -12.021 --start-lon -77.054 \
    --end-lat -12.023 --end-lon -77.055 \
    -o output/segmento.mp4
```

**Características:**
- ⚡ **Rápido**: Usa stream copy (sin re-codificación) cuando es posible
- 🎯 **Preciso**: Encuentra los puntos GPS más cercanos a tus coordenadas
- 💾 **Preserva GPS**: Los datos GPS se mantienen en el segmento

### 5. Ver info de rosbag

```bash
python -m videogps info rosbags/tu_rosbag
```

## Configuración (`config.yaml`)

### Mínima configuración requerida:

```yaml
calibration_path: stereo_calib.json

camera:
  topic_left: /camera_left/image_raw/compressed
  topic_right: /camera_right/image_raw/compressed

gps:
  topic: /swift/navsat_fix

video:
  output_fps: 24
  output_dir: output
  embed_gps_track: true
```

### Habilitar overlay GPS (opcional):

```yaml
overlay:
  enabled: true  # Mostrar GPS en el video
  position: top_left
  show_speed: true
  show_altitude: true
```

## Formato de salida

El video MP4 generado contiene:
- **Video**: Cámaras stereo rectificadas y sincronizadas
- **GPS embebido**: Datos GPS guardados en archivo `.gps.json`
  - Latitud, Longitud, Altitud
  - Timestamp, Número de satélites
  - Número de frame

## Comandos disponibles

```bash
# Generar video
python -m videogps generate <rosbag> --config config.yaml

# Cortar por coordenadas GPS
python -m videogps cut <video.mp4> --start-lat LAT --start-lon LON --end-lat LAT --end-lon LON

# Cortar por tiempo
python -m videogps cut-time <video.mp4> --start 10.0 --end 30.0

# Ver info de rosbag
python -m videogps info <rosbag>

# Crear config
python -m videogps init
    --end-lat 40.7589 --end-lon -73.9851 \
    --tolerance 50

# With custom output name
python -m videogps cut video.mp4 \
    --start-lat 40.7128 --start-lon -74.0060 \
    --end-lat 40.7589 --end-lon -73.9851 \
    -o segment_A_to_B.mp4
```

**How it works:**
1. Reads embedded GPS data from `.gps.json` file
2. Finds closest GPS points to your coordinates (within tolerance)
3. Extracts video segment between those points
4. **No re-encoding** - instant extraction
5. GPS data automatically preserved

### Cut Video Segment by Time (Alternative)

```bash
# Cut from 10s to 30s (if you know exact times)
python -m videogps cut-time video.mp4 --start 10.0 --end 30.0
```

**Note**: GPS data is automatically preserved in both methods!

### Extract GPS Data from Video

```bash
# Extract GPS subtitle track
ffmpeg -i video.mp4 -map 0:s:0 -c copy gps_data.srt

# View as JSON
cat gps_data.srt
```

### Extract Trajectory Segment by Coordinates

```bash
python -m videogps extract-segment --config config.yaml \
    --start-lat 40.7128 --start-lon -74.0060 \
    --end-lat 40.7589 --end-lon -73.9851 \
    --tolerance 50
```

## Configuration

The `config.yaml` file controls all aspects of video generation:

```yaml
# Core paths
rosbag_path: rosbags/my_rosbag
calibration_path: stereo_calib.json

# Camera settings
camera:
  topic_left: /camera_left/image_raw/compressed
  topic_right: /camera_right/image_raw/compressed
  sync_tolerance_ns: 50000000  # 50ms
  rectify_enabled: true
  max_width: 1920

# GPS settings
gps:
  topic: /swift/navsat_fix
  interpolation_enabled: true
  outlier_filter_enabled: true
  min_satellites: 4

# Video output
video:
  output_fps: 24
  codec: avc1  # H.264 for best compatibility
  quality: 95
  output_dir: output
  embed_gps_track: true  # Embed GPS as subtitle track

# GPS overlay (optional, disabled by default)
overlay:
  enabled: false  # Set to true to show GPS on video
  position: top_left
  show_altitude: true
```

## Requirements

- Python 3.8+
- ffmpeg (for GPS embedding and segmentation)
  - Windows: Download from https://ffmpeg.org/download.html
  - Add to PATH or place in project directory
video:
  output_fps: 24
  codec: mp4v
  output_dir: output
```

See `config.yaml` for complete configuration options.

## Architecture

```
videogps/
├── core/               # Core data structures and algorithms
│   ├── config.py       # Configuration management
│   ├── gps_trajectory.py   # GPS trajectory with interpolation
│   └── camera_rectifier.py # Camera undistortion
├── processors/         # Data processing modules
│   ├── rosbag_reader.py    # ROS2 rosbag extraction
│   ├── gps_overlay.py      # Overlay rendering
│   └── video_generator.py  # Main orchestrator
└── utils/              # Utility functions
    ├── logger.py       # Professional logging
    ├── geo_utils.py    # Geographic calculations
    └── time_sync.py    # Stereo synchronization
```

## Key Features Explained

### Stereo Synchronization
Only frames with synchronized stereo pairs (within configurable tolerance) are processed. This ensures high-quality output for stereo vision applications.

### GPS Interpolation
Linear interpolation between GPS measurements provides smooth overlay even when GPS update rate differs from camera frame rate.

### Outlier Filtering
Physics-based filtering removes GPS outliers by checking:
- Maximum realistic speed (default: 50 m/s)
- Maximum acceleration (default: 10 m/s²)
- Minimum satellite count (default: 4)

### Performance Optimizations
- **O(log n)** GPS point lookup using binary search
- **O(1)** camera rectification with pre-computed maps
- **O(1)** overlay rendering with cached measurements
- Batch processing for efficiency

## Requirements

- Python 3.8+
- OpenCV 4.5+
- NumPy 1.20+
- ROS2 rosbag support
- Stereo camera calibration file

## License

MIT License

## Contributing

Contributions welcome! This modular architecture makes it easy to add new features:
- Additional overlay styles
- New trajectory filters
- Alternative video codecs
- Depth map generation
- And more...
