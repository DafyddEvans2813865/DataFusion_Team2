# Python Radar Parser Makefile

PYTHON = python3
TEST_DATA = tests/data/example/Radar_Test_Data.txt
OUTPUT_BAG = radar_output.bag
RADAR_CSV_DIR = tests/data/radar_parsing

# Default target
all: radar_parser

# Parse radar data using Python
radar_parser:
	cd src/radar && $(PYTHON) radar_parser.py

# Run radar tests
test_radar:
	$(PYTHON) -m unittest tests.test_radar -v

# Run all tests
test: test_radar

# Clean up generated files
clean:
	rm -f $(OUTPUT_BAG)
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete
	rm -rf $(RADAR_CSV_DIR)
	rm -f tests/test_radar_*.txt

.PHONY: all radar_parser test test_radar clean