# Python Radar Parser Makefile

PYTHON = python3
PYTHONPATH = src
TEST_DATA = tests/data/example/Radar_Test_Data.txt
OUTPUT_BAG = radar_output.bag
RADAR_CSV_DIR = tests/data/radar_parsing

# Default target
all: radar_parser imu_parser

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
	rm -f $(OUTPUT_BAG)
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete
	rm -rf $(RADAR_CSV_DIR)
	rm -f tests/test_radar_*.txt

.PHONY: all radar_parser test test_radar clean