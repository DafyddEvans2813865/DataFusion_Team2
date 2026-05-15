# DataFusion Build and Test Commands

## Tests

- `make test_radar` - Run radar parser Python unit tests
- `make test_imu` - Run IMU parser Python unit tests
- `make test` - Run all tests (radar + IMU)
- `make clean` - Clean up generated files and cache

## ROS Bag File Conversion

### Convert Both Radar and IMU to Bag Files

- `make` or `make all` - Convert both radar and IMU sensor data to ROS bag files
- `make bags` - Same as above (alias for convert)
- `make convert` - Explicitly convert both radar and IMU data to bag files

This creates:

- `radar_output.bag` - ROS bag file with radar PointCloud2 messages
- `imu_output.bag` - ROS bag file with IMU messages

**Requirements**: ROS must be installed with rosbag support

### Individual Data Processing

- `make radar_parser` - Parse `tests/data/example/Radar_Test_Data.txt` using the Python radar parser (displays parsed data)
- `make imu_parser` - Parse A2 CSV IMU data using the Python IMU parser with CSV verification export (displays parsed data)

## Directory Structure

### Main Module

- `src/main.py` - Main conversion script that generates both radar and IMU bag files

### Radar Module

- `src/radar/` - Python radar parsing module
  - `radar_point.py` - RadarPoint data class
  - `radar_parser_impl.py` - RadarParser implementation (TI IWR format with bag file export)
  - `radar_parser.py` - Module interface and CLI
  - `__init__.py` - Package exports

### IMU Module

- `src/imu/` - Python IMU parsing module
  - `imu_point.py` - IMUPoint data class with full 3D orientation support
  - `imu_parser_impl.py` - IMUParser implementation (A2 CSV format parsing and bag file export)
  - `imu_parser.py` - Module interface and CLI
  - `__init__.py` - Package exports

### Tests

- `tests/` - Test suite
  - `test_radar.py` - Radar parser unit tests
  - `test_imu.py` - IMU parser unit tests (A2 CSV and binary formats)
  - `data/example/` - Test data
    - `Radar_Test_Data.txt` - TI radar hex format
    - `a2_packet_type_a2.csv` - IMU A2 mode CSV format (stationary_A2)
    - `stationary_A2.bin` - IMU A2 mode binary format (2001 packets at ~100 Hz)

## Data Formats

### TI Radar Format (radar_parser_impl.py)

- **Synchronization**: Magic word `0x0201040306050807` (8 bytes)
- **Header**: Version, packet length, platform, frame number, timestamp, numObjects, numTLV, subframeNum, numStaticObjects (36 bytes)
- **TLVs**: Type-Length-Value sections containing detection points
  - Type 1: Dynamic Object Detection Points (range, angle, elevation, doppler as floats)

### IMU A2 Mode CSV Format (imu_parser_impl.py)

OpenIMU300ZI A2 mode (stationary_A2 packet format) CSV data:

**Columns:**

- `timeITOW (msec)` - Time in milliseconds
- `time (s)` - Time in seconds
- `roll, pitch, heading (deg)` - Euler angles in degrees
- `xRate (rad/s)` - X angular rate in radians/sec
- `yRate, zRate (deg/s)` - Y, Z angular rates in degrees/sec
- `xAccel, yAccel, zAccel (m/s²)` - Linear accelerations

The parser converts all angles to radians and rates to rad/s for internal consistency. CSV export format uses degrees/deg-sec for human readability and verification.

### IMU A2 Mode Binary Format (imu_parser_impl.py)

OpenIMU300ZI A2 mode binary data (stationary_A2.bin):

**Packet Structure:**

- **Header (5 bytes)**: `0x55 0x55 0x61 0x32 0x30` (UUa20 in ASCII)
- **Payload (50 bytes)**: 12 float values containing sensor data
- **Total per packet**: 55 bytes
- **Sample rate**: ~100 Hz (10 ms intervals)

**Payload Fields (12 floats):**

- Float 0-1: Reserved/timestamp fields
- Float 2-4: Roll, pitch, yaw (heading) in degrees
- Float 5-7: Angular rates (x, y, z)
- Float 8-10: Linear accelerations (x, y, z) in m/s²
- Float 11: Reserved

The binary parser automatically detects format by file extension (.bin for binary, .csv for CSV) and applies appropriate unit conversions.

## Test Coverage

### Radar Tests (`test_radar.py`)

- RadarPoint creation and conversion
- TI radar format parsing (magic word synchronization, TLV extraction)
- Multiple points per frame
- Hex data with spaces
- Coordinate transformation (spherical to Cartesian)
- Error handling (missing files, invalid data)
- ROS integration (skipped if ROS not installed)

### IMU Tests (`test_imu.py`)

- IMUPoint creation and conversion
- IMU CSV format parsing (header detection, CSV extraction)
- IMU binary format parsing (0x55 header detection, packet extraction)
- Multiple measurements per file
- Unit conversions (degrees to radians, rates)
- Error handling (missing files, invalid data)
- Real A2 test data parsing with orientation verification (both CSV and binary formats)
- Real test data parsing
