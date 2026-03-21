# Compiler
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I./src

# Find all .cpp files recursively in src/
SRCS := $(shell find src -name "*.cpp")

# Convert .cpp to .o
OBJS := $(SRCS:.cpp=.o)

# Executable name
TARGET = data_fusion

# Default target - run C++ tests
all: test_cpp

# Link object files into executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile each .cpp into .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean object files and executables
clean:
	rm -f $(OBJS) $(TARGET) radar_tests imu_tests
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete

# Run C++ tests only
test_cpp: radar_test imu_test

# Radar tests (C++ dummy)
radar_test:
	$(CXX) $(CXXFLAGS) \
	tests/test_radar_dummy.cpp src/dummy/RadarDummy.cpp \
	-o radar_tests
	./radar_tests

# IMU tests
imu_test:
	$(CXX) $(CXXFLAGS) \
	tests/test_imu_dummy.cpp src/imu/IMU.cpp src/imu/IMUBuffer.cpp \
	-o imu_tests
	./imu_tests

.PHONY: all clean test_cpp radar_test imu_test