# DataFusion Build and Test Commands

## Tests

- `make test_radar` - Run radar parser Python unit tests
- `make test` - Run all tests (currently radar tests)
- `make clean` - Clean up generated files and cache

## Radar Data Processing

- `make radar_parser` - Parse `tests/data/example/Radar_Test_Data.txt` using the Python radar parser

## Directory Structure

- `src/radar/` - Python radar parsing module
  - `radar_point.py` - RadarPoint data class
  - `radar_parser_impl.py` - RadarParser implementation
  - `radar_parser.py` - Module interface
  - `__init__.py` - Package exports
- `tests/` - Test suite
  - `test_radar.py` - Radar parser unit tests
  - `data/example/` - Original test data (Radar_Test_Data.txt)
- `src/imu/` - IMU module (C++)
- `src/dummy/` - Dummy data generators (C++)
