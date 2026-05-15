# DataFusion Build and Test Commands

## Tests

- `make test_radar` - Run radar parser Python unit tests
- `make test_imu` - Run IMU parser Python unit tests including multithreading tests
- `make test` - Run all tests (tests first, then conversion) - **DEFAULT TARGET**
- `make clean` - Clean up generated files and cache

## ROS Bag File Conversion

### Convert Both Radar and IMU to Bag Files

- `make bags` - Convert both radar and IMU sensor data to ROS bag files
- `make convert` - Explicitly convert both radar and IMU data to bag files

This creates:

- `radar_output.bag` - ROS bag file with radar PointCloud2 messages
- `imu_output.bag` - ROS bag file with IMU messages (from multithreaded streaming)

**IMU Streaming**: Uses multithreaded architecture for continuous serial input

- Connect OpenIMU300ZI to serial port (default: `/dev/ttyUSB0` @ 115200 baud)
- Press `Ctrl+C` to stop recording and close bag file

**Requirements**: ROS must be installed with rosbag support

### Individual Data Processing

- `make radar_parser` - Parse `tests/data/example/Radar_Test_Data.txt` using the Python radar parser (displays parsed data)
- `make imu_parser` - Parse A2 CSV IMU data using the Python IMU parser with CSV verification export (displays parsed data)

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

- `src/imu/` - Python IMU parsing module with multithreaded streaming
  - `imu_point.py` - IMUPoint data class with quaternion storage (qx, qy, qz, qw)
  - `imu_parser_impl.py` - IMUParser implementation with:
    - `to_bag_multithreaded()` - Main entry point for 3-threaded streaming
    - `_reader_worker()` - Serial reader thread (buffering packets)
    - `_parser_worker()` - Packet parser thread (Euler→Quaternion conversion)
    - `_writer_worker()` - Bag writer thread (disk I/O)
  - `imu_multithread.py` - Reference/educational implementation
  - `__init__.py` - Package exports

### Tests

- `tests/` - Test suite
  - `test_radar.py` - Radar parser unit tests
  - `test_imu.py` - IMU parser unit tests including:
    - IMUPoint quaternion storage tests
    - Payload unpacking tests (uint32 + double + 9 floats)
    - Packet structure validation
    - **Multithreading tests**: Queue operations, thread synchronization, stop events
    - ROS2 message conversion (skipped if ROS2 not installed)
  - `data/example/` - Test data
    - `Radar_Test_Data.txt` - TI radar hex format
    - `IMU_Test_Example.csv` - IMU A2 mode sample data

## Data Formats

### TI Radar Format (radar_parser_impl.py)

- **Synchronization**: Magic word `0x0201040306050807` (8 bytes)
- **Header**: Version, packet length, platform, frame number, timestamp, numObjects, numTLV, subframeNum, numStaticObjects (36 bytes)
- **TLVs**: Type-Length-Value sections containing detection points
  - Type 1: Dynamic Object Detection Points (range, angle, elevation, doppler as floats)

### IMU A2 Mode CSV Format (imu_parser_impl.py)

OpenIMU300ZI A2 mode (stationary_A2 packet format) CSV data:

**Columns:**

- `timeITOW (msec)` - Time in milliseconds
- `time (s)` - Time in seconds
- `roll, pitch, heading (deg)` - Euler angles in degrees
- `xRate (rad/s)` - X angular rate in radians/sec
- `yRate, zRate (deg/s)` - Y, Z angular rates in degrees/sec
- `xAccel, yAccel, zAccel (m/s²)` - Linear accelerations

The parser converts all angles to radians and rates to rad/s for internal consistency. CSV export format uses degrees/deg-sec for human readability and verification.

### IMU A2 Mode Binary Format (imu_parser_impl.py)

OpenIMU300ZI A2 mode binary data (stationary_A2.bin):

**Packet Structure:**

- **Header (5 bytes)**: `0x55 0x55 0x61 0x32 0x30` (UUa20 in ASCII)
- **Payload (50 bytes)**: 12 float values containing sensor data
- **Total per packet**: 55 bytes
- **Sample rate**: ~100 Hz (10 ms intervals)

**Payload Fields (12 floats):**

- Float 0-1: Reserved/timestamp fields
- Float 2-4: Roll, pitch, yaw (heading) in degrees
- Float 5-7: Angular rates (x, y, z)
- Float 8-10: Linear accelerations (x, y, z) in m/s²
- Float 11: Reserved

The binary parser automatically detects format by file extension (.bin for binary, .csv for CSV) and applies appropriate unit conversions.

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
- IMU CSV format parsing (header detection, CSV extraction)
- IMU binary format parsing (0x55 header detection, packet extraction)
- Multiple measurements per file
- Unit conversions (degrees to radians, rates)
- Error handling (missing files, invalid data)
- Real A2 test data parsing with orientation verification (both CSV and binary formats)
- Real test data parsing

## Multithreading Architecture

The IMU parser implements a **3-thread producer-consumer pattern** for efficient continuous streaming:

### Thread Design

```
[Serial Port @ 115200 baud]
        ↓
[Reader Thread]
- Reads 1 byte at a time
- Buffers into 55-byte packets
- Syncs to header (UUa20)
- Puts packets into raw_packet_queue
        ↓
    [Queue: max 1000 packets]
        ↓
[Parser Thread]
- Gets raw packets from queue
- Unpacks payload: uint32 (seq) + double (time) + 9 floats (data)
- Converts Euler angles to quaternion
- Creates ROS2 Imu message
- Serializes with CDR format
- Puts (serialized_msg, timestamp_ns) into msg_queue
        ↓
    [Queue: max 500 messages]
        ↓
[Writer Thread]
- Gets serialized messages from queue
- Writes to rosbag2 SequentialWriter
- Maintains message ordering via timestamps
- Logs progress every 100 packets

        ↓
[ROS2 Bag File]
```

### Performance Benefits

- **~2x throughput improvement** over single-threaded approach
- **Overlapping I/O**: Serial reads don't block parsing; parsing doesn't block disk writes
- **Graceful shutdown**: `Ctrl+C` sets stop_event, all threads exit cleanly
- **Bounded queues**: Prevent unbounded memory growth during long sessions

### Configuration

In `_reader_worker()` and `_writer_worker()`:

- `raw_packet_queue = queue.Queue(maxsize=1000)` - 55 bytes × 1000 = ~55 KB
- `msg_queue = queue.Queue(maxsize=500)` - ~100-150 KB total
- `timeout=1` - 1 second timeout on queue operations (allows stop event checking)

### Thread Safety

- Python `queue.Queue` provides thread-safe FIFO operations
- `threading.Event` used for graceful shutdown signal
- No shared mutable state beyond queues
- All threads daemonized: `daemon=False` ensures clean shutdown via `join()`

### Orientation Storage

**Old approach**: Store Euler angles (roll, pitch, yaw) → convert to quaternion on each message creation
**New approach**: Store quaternion directly in IMUPoint (qx, qy, qz, qw) → use directly in ROS message

Benefits:

- No repeated trig conversions (quaternion computed once during parsing)
- ROS2-native format (Imu message uses Quaternion)
- More efficient for 30+ minute streams
- Still supports Euler-to-quaternion conversion via `_quaternion_from_euler()`
