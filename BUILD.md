# DataFusion Build Commands

## Main Build

- `make` - Build the main executable (`data_fusion`)
- `make clean` - Remove all compiled objects and executables

## Tests

- `make imu_test` - Run IMU CSV format tests
- `make radar_test` - Run radar dummy data tests
- `make test_all` - Run all tests (IMU and radar dummy)

## Radar Data Processing

- `make radar_parser` - Parse `tests/data/example/Radar_Test_Data.txt` and generate CSV files in `tests/data/radar_parsing/`:
  - `dynamic_points.csv` - Detected radar points with SNR/noise
  - `static_points.csv` - Static radar points with doppler
  - `tracks.csv` - Tracked objects with position/velocity/acceleration
  - `associations.csv` - Point-to-track associations

## Directory Structure

- `tests/data/example/` - Original test data (IMU_Test_Data.csv, Radar_Test_Data.txt)
- `tests/data/radar_parsing/` - Generated radar CSV outputs
