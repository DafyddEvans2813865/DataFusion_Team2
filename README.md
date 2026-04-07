# DataFusion Team 2

A Python-based sensor data processing system for parsing radar and IMU data, with support for ROS bag file export.

## Overview

This project provides tools for:

- Parsing TI IWR radar data in TLV format with magic word synchronization
- Parsing binary IMU data with 0x5555 packet headers
- Converting spherical (range, angle, elevation) to Cartesian (x, y, z) coordinates
- Exporting radar data to ROS bag files for use with ROS-based systems
- Comprehensive unit testing of sensor data parsing

## Project Structure

```
DataFusion_Team2/
├── src/
│   ├── radar/              # Python radar parsing module
│   │   ├── __init__.py
│   │   ├── radar_point.py          # RadarPoint dataclass
│   │   ├── radar_parser_impl.py    # Core TI radar parser (TLV format)
│   │   └── radar_parser.py         # Module interface
│   ├── imu/                # Python IMU parsing module
│   │   ├── __init__.py
│   │   ├── imu_point.py            # IMUPoint dataclass
│   │   ├── imu_parser_impl.py      # Core IMU parser (binary format)
│   │   └── imu_parser.py           # Module interface
│   └── main.cpp            # C++ main (legacy)
├── tests/
│   ├── test_radar.py               # Radar parser unit tests
│   └── data/
│       └── example/
│           ├── Radar_Test_Data.txt # TI radar hex format
│           └── 20_second_moving_test.bin # IMU binary data
├── Makefile                # Build and test commands
├── requirements.txt        # Python dependencies
├── BUILD.md               # Detailed build documentation
└── README.md              # This file
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
make test_imu        # Run IMU parser tests
make test           # Run all tests
```

### Parse Sensor Data

```bash
make                 # Parse both radar and IMU data
make radar_parser    # Parse radar data only
make imu_parser      # Parse IMU data only
```

Or in Python:

```python
from radar import RadarParser
from imu import IMUParser

# Parse radar data
radar_parser = RadarParser()
radar_points = radar_parser.parse_hex_text('tests/data/example/Radar_Test_Data.txt')

# Export to ROS bag (requires ROS)
radar_parser.to_bag('radar_output.bag', radar_points)

# Parse IMU data
imu_parser = IMUParser()
imu_points = imu_parser.parse_binary_file('tests/data/example/20_second_moving_test.bin')

# Access IMU data
if imu_points:
    for point in imu_points:
        print(f"Time: {point.time}, Roll: {point.roll}, Pitch: {point.pitch}")
```

### Cleanup

```bash
make clean          # Remove generated files and cache
```

## API

### Radar Module

#### RadarPoint

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

#### RadarParser

**Methods:**

- `parse_hex_text(hex_text_path: str) -> List[RadarPoint]`
  - Parses TI IWR radar data from hex-encoded text file
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

### IMU Module

#### IMUPoint

Data class representing a single IMU measurement.

**Fields:**

- `time_counter` (int): Time counter value
- `time` (float): Time in seconds (double precision)
- `roll`, `pitch`, `heading` (float): Euler angles in degrees
- `x_accel`, `y_accel`, `z_accel` (float): Acceleration in g's
- `x_rate`, `y_rate`, `z_rate` (float): Angular rates in deg/s
- `x_rate_bias`, `y_rate_bias`, `z_rate_bias` (float): Angular rate biases
- `x_mag`, `y_mag`, `z_mag` (float): Magnetometer readings
- `op_mode`, `lin_acc_switch`, `turn_switch` (int): Mode flags

#### IMUParser

**Methods:**

- `parse_binary_file(binary_file_path: str) -> List[IMUPoint]`
  - Parses binary IMU data with 0x5555 packet headers
  - Returns list of IMUPoint objects

## Testing

The project includes comprehensive unit tests for sensor data parsing:

```bash
make test_radar     # Run radar parser unit tests
make test_imu       # Run IMU parser unit tests
make test          # Run all tests (radar + IMU)
```

**Radar Test Coverage:**

- RadarPoint creation and conversion
- TI radar format parsing (magic word synchronization, TLV extraction)
- Multiple points per frame
- Hex data with spaces
- Coordinate transformation (spherical to Cartesian)
- Error handling (missing files, invalid data)
- ROS integration (skipped if ROS not installed)

**IMU Test Coverage:**

- IMUPoint creation and conversion
- IMU binary format parsing (0x5555 header detection, packet extraction)
- Multiple measurements per file
- Error handling (missing files, invalid data)
- Real test data parsing

## Data Formats

### TI Radar Format

Binary format with:

- **Magic Word**: `0x0201040306050807` (8 bytes) for frame synchronization
- **Header**: 36 bytes including version, packet length, frame number, timestamp
- **TLVs**: Type-Length-Value sections for different data types
  - Type 1: Dynamic Object Detection Points (4 floats per point: range, angle, elevation, doppler)

### IMU Binary Format

Packet-based format with:

- **Header**: `0x5555` (2 bytes, little-endian: 0x55, 0x55)
- **Packet Structure**: `[header(2)][type(2)][length(1)][payload(length)][checksum(2)]`
- **Payload**: 19 fields total including time, attitude, accelerations, rates, magnetometer readings

## CI/CD

Tests automatically run on push and pull requests via GitHub Actions. The workflow:

1. Sets up Python 3.10
2. Installs dependencies from requirements.txt
3. Runs all unit tests

## Notes

- ROS/rosbag functionality is optional. Core radar parsing works without ROS.
- IMU parsing is independent of ROS and always available.
- Tests that require ROS are automatically skipped if not installed.
- All sensor data formats use little-endian byte order.

## License

TBD
