# Python Radar and IMU Parser Makefile

PYTHON = python3
PYTHONPATH = src
TEST_DATA = tests/data/example/Radar_Test_Data.txt
RADAR_OUTPUT_BAG = radar_output.bag
IMU_OUTPUT_BAG = imu_output.bag
RADAR_CSV_DIR = tests/data/radar_parsing

# Default target
all: bags

# Convert both radar and IMU data to ROS bag files
bags: convert

convert:
	CWD=$(PWD) $(PYTHON) src/main.py

# Parse radar data using Python
radar_parser:
	PYTHONPATH=$(PYTHONPATH) $(PYTHON) -m radar.radar_parser

# Parse IMU data using Python
imu_parser:
	PYTHONPATH=$(PYTHONPATH) $(PYTHON) -m imu.imu_parser

# Run radar tests
test_radar:
	PYTHONPATH=$(PYTHONPATH) $(PYTHON) -m unittest tests.test_radar -v

# Run IMU tests
test_imu:
	PYTHONPATH=$(PYTHONPATH) $(PYTHON) -m unittest tests.test_imu -v

# Run all tests
test: test_radar test_imu

# Clean up generated files
clean:
	rm -f $(RADAR_OUTPUT_BAG) $(IMU_OUTPUT_BAG)
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete
	rm -rf $(RADAR_CSV_DIR)
	rm -f tests/test_radar_*.txt

.PHONY: all bags convert radar_parser imu_parser test test_radar test_imu clean