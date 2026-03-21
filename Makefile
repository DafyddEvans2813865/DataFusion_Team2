# Compiler
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I./src

# Find all .cpp files recursively in src/
SRCS := $(shell find src -name "*.cpp")

# Convert .cpp to .o
OBJS := $(SRCS:.cpp=.o)

# Executable name
TARGET = data_fusion

# Default target - run tests first, then build
all: test_all $(TARGET)


# Link object files into executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile each .cpp into .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean object files and executables
clean:
	rm -f $(OBJS) $(TARGET) radar_tests imu_tests radar_parser

# Run all tests
test_all: radar_test imu_test

# Radar tests
radar_test:
	$(CXX) $(CXXFLAGS) \
	tests/test_radar_dummy.cpp src/dummy/RadarDummy.cpp \
	-o radar_tests
	./radar_tests

# Radar parser test
radar_parser:
	$(CXX) $(CXXFLAGS) \
	tests/test_radar_parser.cpp src/radar/RadarParser.cpp \
	-o radar_parser
	./radar_parser

# IMU tests
imu_test:
	$(CXX) $(CXXFLAGS) \
	tests/test_imu_dummy.cpp src/imu/IMU.cpp src/imu/IMUBuffer.cpp \
	-o imu_tests
	./imu_tests

# Legacy test target (runs radar tests only)
test:
	$(CXX) $(CXXFLAGS) \
	tests/test_radar_dummy.cpp src/dummy/RadarDummy.cpp \
	-o radar_tests
	./radar_tests