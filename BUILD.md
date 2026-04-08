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
- `make imu_parser` - Parse IMU binary data using the Python IMU parser (displays parsed data)

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
  - `imu_point.py` - IMUPoint data class
  - `imu_parser_impl.py` - IMUParser implementation (binary format with 0x5555 headers and bag file export)
  - `imu_parser.py` - Module interface and CLI
  - `__init__.py` - Package exports

### Tests

- `tests/` - Test suite
  - `test_radar.py` - Radar parser unit tests
  - `test_imu.py` - IMU parser unit tests
  - `data/example/` - Test data
    - `Radar_Test_Data.txt` - TI radar hex format
    - `IMU_Test_Data.csv` - IMU CSV data
    - `IMU_Test_Data.bin` - IMU binary data (0x5555 packet format)

## Data Formats

### TI Radar Format (radar_parser_impl.py)

- **Synchronization**: Magic word `0x0201040306050807` (8 bytes)
- **Header**: Version, packet length, platform, frame number, timestamp, numObjects, numTLV, subframeNum, numStaticObjects (36 bytes)
- **TLVs**: Type-Length-Value sections containing detection points
  - Type 1: Dynamic Object Detection Points (range, angle, elevation, doppler as floats)

### IMU Binary Format (imu_parser_impl.py)

- **Header**: `0x5555` (2 bytes)
- **Packet Structure**: `[0x55 0x55][type(2)][length(1)][payload][checksum(2)]`
- **Data Fields**: 19 fields including time, attitude (roll/pitch/heading), accelerations, rates, biases, magnetometer, and mode flags

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
- IMU binary format parsing (0x5555 header detection, packet extraction)
- Multiple measurements per file
- Error handling (missing files, invalid data)
- Real test data parsing
