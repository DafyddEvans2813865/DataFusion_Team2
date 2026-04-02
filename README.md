# DataFusion Team 2

A Python-based radar data processing system for parsing hex-encoded radar point data and converting it to ROS bag files.

## Overview

This project provides tools for:
- Parsing hex-encoded radar data into structured point clouds
- Converting spherical (range, angle, elevation) to Cartesian (x, y, z) coordinates
- Exporting radar data to ROS bag files for use with ROS-based systems
- Comprehensive unit testing of radar parsing functionality

## Project Structure

```
DataFusion_Team2/
├── src/
│   ├── radar/              # Python radar parsing module
│   │   ├── __init__.py
│   │   ├── radar_point.py          # RadarPoint dataclass
│   │   ├── radar_parser_impl.py    # Core parser implementation
│   │   └── radar_parser.py         # Module interface
│   └── imu/                # IMU module (C++)
├── tests/
│   ├── test_radar.py               # Radar parser unit tests
│   └── data/
│       └── example/
│           └── Radar_Test_Data.txt # Sample radar data
├── Makefile                # Build and test commands
├── requirements.txt        # Python dependencies
└── BUILD.md               # Detailed build documentation
```

## Requirements

- Python 3.10+
- Dependencies: numpy (and optionally rosbag for bag file export)

## Installation

### Install Python dependencies:
```bash
pip install -r requirements.txt
```

### For ROS integration (optional):
Install ROS and rosbag to enable bag file export functionality.

## Usage

### Run Tests
```bash
make test_radar      # Run radar parser tests
make test           # Run all tests
```

### Parse Radar Data
```bash
make radar_parser   # Parse example radar data
```

Or in Python:
```python
from radar import RadarParser

parser = RadarParser()
points = parser.parse_hex_text('tests/data/example/Radar_Test_Data.txt')

# Export to ROS bag (requires ROS)
parser.to_bag('output.bag', points)

# Convert to PointCloud2 (requires ROS)
point_cloud = parser.to_point_cloud(points)
```

### Cleanup
```bash
make clean          # Remove generated files and cache
```

## API

### RadarPoint
Data class representing a single radar detection.

**Fields:**
- `frame` (int): Frame number
- `timestamp` (int): Timestamp in milliseconds
- `range` (float): Distance in meters
- `angle` (float): Azimuth angle in degrees
- `elev` (float): Elevation angle in degrees
- `doppler` (float): Doppler velocity
- `snr` (int): Signal-to-noise ratio
- `noise` (int): Noise level

### RadarParser

**Methods:**
- `parse_hex_text(hex_text_path: str) -> List[RadarPoint]`
  - Parses hex-encoded radar data from a text file
  - Returns list of RadarPoint objects

- `to_point_cloud(points: List[RadarPoint], frame_id: str) -> PointCloud2`
  - Converts radar points to ROS PointCloud2 message
  - Requires ROS installation
  - Returns None if ROS not available

- `to_bag(output_path: str, topic_name: str) -> bool`
  - Exports parsed radar data to ROS bag file
  - Requires ROS installation
  - Returns True on success, False otherwise

- `inspect_bag(bag_path: str) -> None`
  - Prints information about a ROS bag file
  - Requires ROS installation

## Testing

The project includes comprehensive unit tests for the radar parsing module:

```bash
python3 -m unittest tests.test_radar -v
```

**Test Coverage:**
- RadarPoint creation and conversion
- Hex data parsing (valid, multiple points, with spaces)
- Coordinate transformation (spherical to Cartesian)
- Error handling (missing files, invalid data)
- ROS integration (skipped if ROS not installed)

## CI/CD

Tests automatically run on push and pull requests via GitHub Actions. The workflow:
1. Sets up Python 3.10
2. Installs dependencies from requirements.txt
3. Runs radar parser unit tests

## Notes

- ROS/rosbag functionality is optional. The core parsing works without ROS.
- Tests that require ROS are automatically skipped if not installed.
- Hex data format: 20 bytes per point (5 x 4-byte floats/ints: frame, timestamp, range, angle, doppler)

## License

TBD
