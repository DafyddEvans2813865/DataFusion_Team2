# DataFusion Team 2

A Python-based sensor data processing system for parsing radar and IMU data, with support for ROS bag file export.

## Overview

This project provides tools for:

- Parsing TI IWR radar data in TLV format with magic word synchronization
- Parsing OpenIMU300ZI A2 mode CSV data with orientation information
- Converting spherical (range, angle, elevation) to Cartesian (x, y, z) coordinates
- Exporting sensor data to ROS bag files for use with ROS-based systems
- Comprehensive unit testing of sensor data parsing

## Project Structure

```
DataFusion_Team2/
├── src/
│   ├── main.py             # Main conversion script (radar + IMU to bag files)
│   ├── radar/              # Python radar parsing module
│   │   ├── __init__.py
│   │   ├── radar_point.py          # RadarPoint dataclass
│   │   ├── radar_parser_impl.py    # Core TI radar parser with bag export
│   │   └── radar_parser.py         # Module interface
│   ├── imu/                # Python IMU parsing module
│   │   ├── __init__.py
│   │   ├── imu_point.py            # IMUPoint dataclass
│   │   ├── imu_parser_impl.py      # Core IMU parser with bag export
│   │   └── imu_parser.py           # Module interface
├── tests/
│   ├── test_radar.py               # Radar parser unit tests
│   ├── test_imu.py                 # IMU parser unit tests
│   └── data/
│       └── example/
│           ├── Radar_Test_Data.txt      # TI radar hex format
│           └── a2_packet_type_a2.csv  # IMU A2 mode CSV format
├── Makefile                # Build and test commands
├── requirements.txt        # Python dependencies
├── BUILD.md               # Detailed build documentation
├── README.md              # This file
└── imu_to_bag.ipynb       # Jupyter notebook: IMU to bag conversion guide
```

## Requirements

- Python 3.10+
- Core dependencies: numpy (installed via pip)
- Optional: ROS 2 for bag file export functionality

## Installation

### Install Python Dependencies

```bash
pip install -r requirements.txt
```

## Usage

### Convert Sensor Data to ROS Bag Files

Convert both radar and IMU sensor data to ROS bag files with a single command:

```bash
make              # Create both radar_output.bag and imu_output.bag
python3 src/main.py  # Same as above
```

This generates two bag files:

- **radar_output.bag**: ROS PointCloud2 messages with radar detections
- **imu_output.bag**: ROS Imu messages with inertial measurements

After conversion, you can use the bag files with ROS tools:

```bash
rosbag play radar_output.bag  # Play radar data
rosbag play imu_output.bag    # Play IMU data
rviz2                         # Visualize in RViz
```

### Run Tests

```bash
make test_radar      # Run radar parser tests
make test_imu        # Run IMU parser tests
make test           # Run all tests
```

### Parse Sensor Data Individually

```bash
make radar_parser    # Parse radar data (display results)
make imu_parser      # Parse IMU data (display results)
```

### Cleanup

```bash
make clean          # Remove generated files and cache
```

### Use in Python Code

```python
from radar.radar_parser_impl import RadarParser
from imu.imu_parser_impl import IMUParser

# Parse radar data
radar_parser = RadarParser()
radar_points = radar_parser.parse_hex_text('tests/data/example/Radar_Test_Data.txt')
print(f'Parsed {len(radar_points)} radar points')

# Export radar to ROS bag (requires ROS)
radar_parser.to_bag('radar_output.bag', '/radar/points')
parser.inspect_bag('radar_output.bag')

# Parse IMU A2 CSV data
imu_parser = IMUParser()
imu_points = imu_parser.parse_a2_csv_file('tests/data/example/a2_packet_type_a2.csv')
print(f'Parsed {len(imu_points)} IMU points from CSV')

# OR Parse IMU A2 binary data
imu_parser = IMUParser()
imu_points = imu_parser.parse_a2_binary_file('tests/data/example/stationary_A2.bin')
print(f'Parsed {len(imu_points)} IMU points from binary')

# Export parsed data to CSV for verification
imu_parser.to_csv('imu_parsed_output.csv')

# Export IMU to ROS bag (requires ROS)
imu_parser.to_bag('imu_output.bag', '/imu/data')
imu_parser.inspect_bag('imu_output.bag')

# Access individual data
if imu_points:
    for point in imu_points:
        print(f"Time: {point.time}s, Accel: ({point.x_accel:.2f}, {point.y_accel:.2f}, {point.z_accel:.2f}) m/s²")
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

- `to_bag(output_path: str, topic_name: str = "/radar/points") -> bool`
  - Exports parsed radar data to ROS bag file
  - Requires ROS installation
  - Returns True on success, False otherwise

- `inspect_bag(bag_path: str) -> None`
  - Prints information about a ROS bag file (duration, topics, sample messages)
  - Requires ROS installation

### IMU Module

#### IMUPoint

Data class representing a single IMU measurement.

**Fields:**

- `time_counter` (int): Time counter value (milliseconds)
- `time` (float): Time in seconds (double precision)
- `roll`, `pitch`, `yaw` (float): Euler angles in radians
- `x_accel`, `y_accel`, `z_accel` (float): Linear acceleration in m/s²
- `x_rate`, `y_rate`, `z_rate` (float): Angular rates in radians/sec
- `x_rate_bias`, `y_rate_bias`, `z_rate_bias` (float): Angular rate biases (unused in A2)
- `x_mag`, `y_mag`, `z_mag` (float): Magnetometer readings (unused in A2)
- `op_mode`, `lin_acc_switch`, `turn_switch` (int): Mode flags (unused in A2)
- `data_source` (str): Data format source (always "A2")

#### IMUParser

**Methods:**

- `parse_a2_csv_file(csv_file_path: str) -> List[IMUPoint]`
  - Parses OpenIMU300ZI A2 mode CSV data (stationary_A2 packet format)
  - Handles unit conversions (degrees → radians)
  - Returns list of IMUPoint objects with full 3D orientation

- `parse_a2_binary_file(binary_file_path: str) -> List[IMUPoint]`
  - Parses OpenIMU300ZI A2 mode binary data (stationary_A2.bin format)
  - Binary packet format: 55-byte packets with 5-byte header ("UUa20") + 50-byte payload
  - Each packet contains sensor readings at approximately 100 Hz sample rate
  - Handles floating-point data extraction and unit conversions
  - Returns list of IMUPoint objects with data_source="A2_BIN"
  - Supports both CSV and binary formats transparently (auto-detection by file extension)

- `to_csv(output_file: str) -> bool`
  - Exports parsed IMU points to CSV format for verification
  - Outputs angles in degrees, rates in deg/s for human readability
  - Includes data_source column
  - Returns True on success, False otherwise

- `to_imu_message(point: IMUPoint, frame_id: str = "imu") -> Imu`
  - Converts single IMUPoint to ROS Imu message
  - Requires ROS installation
  - Orientation includes full 3D pose from yaw angle
  - Returns None if ROS not available

- `to_bag(output_path: str, topic_name: str = "/imu/data") -> bool`
  - Exports parsed IMU data to ROS bag file with Imu messages
  - Requires ROS installation
  - Returns True on success, False otherwise

- `inspect_bag(bag_path: str) -> None`
  - Prints information about a ROS bag file (duration, topics, sample messages)
  - Requires ROS installation

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
- IMU A2 CSV format parsing (header detection, data extraction)
- Unit conversions (degrees to radians)
- Error handling (missing files, invalid data)
- Real A2 test data parsing with orientation verification

## ROS Message Formats

### Radar to PointCloud2

Radar detection data is converted to ROS `sensor_msgs/PointCloud2` messages:

- **Fields**: x, y, z (meters, Cartesian coordinates), intensity (SNR)
- **Frame ID**: "radar"
- **Coordinate system**: Right-handed, x-forward, y-left, z-up

### IMU to Imu Message

IMU sensor data is converted to ROS `sensor_msgs/Imu` messages:

- **Header**: Timestamp and frame ID ("imu")
- **Orientation**: Full 3D orientation from roll, pitch, yaw Euler angles (quaternion)
- **Angular Velocity**: x, y, z (rad/sec)
- **Linear Acceleration**: x, y, z (m/s²)
- **Covariance Matrices**: 3x3 matrices for orientation, angular velocity, and linear acceleration

## Data Formats

### TI Radar Format

Binary format with:

- **Magic Word**: `0x0201040306050807` (8 bytes) for frame synchronization
- **Header**: 36 bytes including version, packet length, frame number, timestamp
- **TLVs**: Type-Length-Value sections for different data types
  - Type 1: Dynamic Object Detection Points (4 floats per point: range, angle, elevation, doppler)

### IMU A2 Mode CSV Format

OpenIMU300ZI A2 mode (stationary_A2 packet format) CSV with columns:

- **timeITOW (msec)**: Time in milliseconds
- **time (s)**: Time in seconds (floating point)
- **roll, pitch, heading (deg)**: Euler angles in degrees
- **xRate (rad/s)**: X-axis angular rate in radians/sec
- **yRate, zRate (deg/s)**: Y and Z angular rates in degrees/sec
- **xAccel, yAccel, zAccel (m/s²)**: Linear accelerations

The parser automatically converts angles to radians and rates to consistent units (rad/s) for internal processing. CSV export provides human-readable degrees/deg-sec format for verification.

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
