# Compiler
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I./src

# Find all .cpp files recursively in src/
SRCS := $(shell find src -name "*.cpp")

# Convert .cpp to .o
OBJS := $(SRCS:.cpp=.o)

# Executable name
TARGET = data_fusion

# Default target
all: $(TARGET)


# Link object files into executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile each .cpp into .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean object files and executable
clean:
	rm -f $(OBJS) $(TARGET)


# Tests
test:
	$(CXX) $(CXXFLAGS) \
	tests/test_radar_dummy.cpp src/dummy/RadarDummy.cpp \
	-o radar_tests