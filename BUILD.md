# DataFusion Build and Test Commands

## Tests

- `make test_radar` - Run radar parser Python unit tests
- `make test_imu` - Run IMU parser Python unit tests
- `make test` - Run all tests (radar + IMU)
- `make clean` - Clean up generated files and cache

## Data Processing

### Radar Data Processing

- `make radar_parser` - Parse `tests/data/example/Radar_Test_Data.txt` using the Python radar parser

### IMU Data Processing

- `make imu_parser` - Parse IMU binary data using the Python IMU parser

## Build All

- `make` or `make all` - Runs both radar and IMU parsers

## Directory Structure

### Radar Module

- `src/radar/` - Python radar parsing module
  - `radar_point.py` - RadarPoint data class
  - `radar_parser_impl.py` - RadarParser implementation (TI IWR format)
  - `radar_parser.py` - Module interface and CLI
  - `__init__.py` - Package exports

### IMU Module

- `src/imu/` - Python IMU parsing module
  - `imu_point.py` - IMUPoint data class
  - `imu_parser_impl.py` - IMUParser implementation (binary format with 0x5555 headers)
  - `imu_parser.py` - Module interface and CLI
  - `__init__.py` - Package exports
  - `IMU.cpp`, `IMU.h` - Legacy C++ code (deprecated)

### Tests

- `tests/` - Test suite
  - `test_radar.py` - Radar parser unit tests
  - `data/example/` - Test data
    - `Radar_Test_Data.txt` - TI radar hex format
    - `20_second_moving_test.bin` - IMU binary data

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
