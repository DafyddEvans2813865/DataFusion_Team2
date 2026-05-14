import struct
import csv
import math
import numpy as np
from pathlib import Path
from typing import List, Optional

from .imu_point import IMUPoint

try:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Quaternion, Vector3
    from std_msgs.msg import Header
    import rclpy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Imu = None
    Quaternion = None
    Vector3 = None
    Header = None
    rclpy = None


class IMUParser:
    # IMU packet structure: [0x55 0x55][type(2)][length(1)][payload][checksum(2)]
    IMU_FMT = '<IdfffffffffffffffBBB'
    IMU_HEADER = b'\x55\x55'
    
    def __init__(self):
        self.points: List[IMUPoint] = []
        self.imu_size = struct.calcsize(self.IMU_FMT)
        self.imu_fields = [
            'time_counter', 'time',
            'roll', 'pitch', 'yaw',
            'x_accel', 'y_accel', 'z_accel',
            'x_rate', 'y_rate', 'z_rate',
            'x_rate_bias', 'y_rate_bias', 'z_rate_bias',
            'x_mag', 'y_mag', 'z_mag',
            'op_mode', 'lin_acc_switch', 'turn_switch'
        ]
    
    # A2 packet constants
    A2_PACKET_TYPE = b'a2'
    A2_PAYLOAD_LEN = 0x30  # 48 bytes: uint32 seq + double time + 9x float32

    @staticmethod
    def _calc_crc(data: bytes) -> int:
        """CRC-CCITT (0x1D0F) used by OpenIMU A2 packets."""
        crc = 0x1D0F
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
            crc &= 0xFFFF
        return crc

    @staticmethod
    def _parse_a2_payload(payload: bytes) -> dict:
        """
        A2 payload = 48 bytes, little-endian:
          bytes 0-3  : uint32  sequence counter
          bytes 4-11 : double  GPS time (seconds)
          bytes 12-47: 9x float32  roll, pitch, yaw, xRate, yRate, zRate,
                                   xAccel, yAccel, zAccel
        """
        seq,  = struct.unpack_from('<I', payload, 0)
        t,    = struct.unpack_from('<d', payload, 4)
        vals  = struct.unpack_from('<9f', payload, 12)
        return {
            'seq':    seq,
            'time':   t,
            'roll':   vals[0],
            'pitch':  vals[1],
            'yaw':    vals[2],
            'xRate':  vals[3],
            'yRate':  vals[4],
            'zRate':  vals[5],
            'xAccel': vals[6],
            'yAccel': vals[7],
            'zAccel': vals[8],
        }

    def _sync_to_packet(self, ser) -> bool:
        """Scan the serial stream until the 0x55 0x55 sync bytes are found."""
        b = ser.read(1)
        if b and b[0] == 0x55:
            b = ser.read(1)
            if b and b[0] == 0x55:
                return True
        return False

    def _read_a2_packet(self, ser):
        """Read one A2 packet from serial (call after sync bytes consumed)."""
        ptype = ser.read(2)
        if len(ptype) < 2 or ptype != self.A2_PACKET_TYPE:
            return None

        plen_b = ser.read(1)
        if not plen_b or plen_b[0] != self.A2_PAYLOAD_LEN:
            return None

        payload = ser.read(self.A2_PAYLOAD_LEN)
        if len(payload) < self.A2_PAYLOAD_LEN:
            return None

        crc_b = ser.read(2)
        if len(crc_b) < 2:
            return None

        crc_rx   = struct.unpack('>H', crc_b)[0]
        crc_calc = self._calc_crc(ptype + plen_b + payload)
        if crc_calc != crc_rx:
            return None

        return self._parse_a2_payload(payload)

    def parse_binary_file(self, IMU_PORT: str, IMU_BAUD: int,
                          duration: Optional[float] = None,
                          timeout: float = 1.0) -> List[IMUPoint]:
        """Stream A2 packets from a live serial port and return parsed IMUPoints.

        Args:
            IMU_PORT:  Serial port, e.g. '/dev/ttyUSB0'
            IMU_BAUD:  Baud rate, e.g. 115200
            duration:  Stop automatically after this many seconds (None = run
                       until KeyboardInterrupt).
            timeout:   Per-read serial timeout in seconds.
        """
        import serial as _serial
        import time as _time

        points  = []
        errors  = 0
        deadline = _time.monotonic() + duration if duration is not None else None

        try:
            with _serial.Serial(IMU_PORT, IMU_BAUD, timeout=timeout) as ser:
                ser.reset_input_buffer()
                print(f"Streaming from {IMU_PORT} at {IMU_BAUD} baud"
                      + (f" for {duration:.0f}s …" if duration else " (Ctrl-C to stop) …"))

                while True:
                    if deadline is not None and _time.monotonic() >= deadline:
                        print(f"\nCapture complete ({duration:.0f}s).")
                        break

                    if not self._sync_to_packet(ser):
                        errors += 1
                        continue

                    pkt = self._read_a2_packet(ser)
                    if pkt is None:
                        errors += 1
                        continue

                    point = IMUPoint(
                        time_counter=pkt['seq'],
                        time=pkt['time'],
                        roll=pkt['roll'],
                        pitch=pkt['pitch'],
                        yaw=pkt['yaw'],
                        x_accel=pkt['xAccel'],
                        y_accel=pkt['yAccel'],
                        z_accel=pkt['zAccel'],
                        x_rate=pkt['xRate'],
                        y_rate=pkt['yRate'],
                        z_rate=pkt['zRate'],
                        x_rate_bias=0.0,
                        y_rate_bias=0.0,
                        z_rate_bias=0.0,
                        x_mag=0.0,
                        y_mag=0.0,
                        z_mag=0.0,
                        op_mode=0,
                        lin_acc_switch=0,
                        turn_switch=0,
                    )
                    points.append(point)

        except KeyboardInterrupt:
            print(f"\nStopped by user.")
        except Exception as e:
            print(f"Error reading from serial port: {e}")
            return []

        print(f"Captured {len(points):,} packets | Errors: {errors:,}")
        self.points = points
        return points



    def parse_a2_binary_file(self, binary_file_path: str) -> List[IMUPoint]:
        """Parse A2 mode binary file with stationary_A2 packet format.
        
        Binary format: Each packet is 55 bytes
        - Header (5 bytes): 0x55 0x55 0x61 0x32 0x30 ("UUa20")
        - Payload (50 bytes): Sensor data in binary format
        
        Payload structure (50 bytes = 12 floats + 2 bytes):
        - Float 0: Reserved/timestamp_ms (converted to time_counter)
        - Float 1-11: Data fields (roll, pitch, heading, rates, accels, etc.)
        """
        points = []
        
        try:
            with open(binary_file_path, 'rb') as f:
                binary_data = f.read()
            
            # Each packet is 55 bytes: 5-byte header + 50-byte payload
            packet_size = 55
            header = b'UUa20'
            
            offset = 0
            packet_num = 0
            
            while offset + packet_size <= len(binary_data):
                # Check for valid header
                if binary_data[offset:offset+5] != header:
                    offset += 1
                    continue
                
                try:
                    # Parse payload (50 bytes = 12 floats + 2 padding bytes)
                    payload_start = offset + 5
                    payload = binary_data[payload_start:payload_start+48]
                    
                    if len(payload) < 48:
                        break
                    
                    # Unpack 12 float values
                    values = struct.unpack('<12f', payload)
                    
                    # Map values to IMU fields
                    # Based on observed data: values[5] ≈ heading, values[11] ≈ zAccel
                    # Mapping is: [reserved, reserved, roll, pitch, ?, heading, xRate, yRate, zRate, xAccel, yAccel, zAccel]
                    roll_deg = values[2]
                    pitch_deg = values[3]
                    heading_deg = values[5]
                    x_rate = values[6]
                    y_rate = values[7]
                    z_rate = values[8]
                    x_accel = values[9]
                    y_accel = values[10]
                    z_accel = values[11]
                    
                    # Generate time counter from packet number (assuming ~100 Hz)
                    time_counter = 209130 + (packet_num * 10)  # 10 ms per packet
                    time = time_counter / 1000.0
                    
                    # Convert angles from degrees to radians
                    roll = math.radians(roll_deg)
                    pitch = math.radians(pitch_deg)
                    yaw = math.radians(heading_deg)
                    
                    # Create IMUPoint
                    point = IMUPoint(
                        time_counter=time_counter,
                        time=time,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw,
                        x_accel=x_accel,
                        y_accel=y_accel,
                        z_accel=z_accel,
                        x_rate=x_rate,
                        y_rate=y_rate,
                        z_rate=z_rate,
                        x_rate_bias=0.0,
                        y_rate_bias=0.0,
                        z_rate_bias=0.0,
                        x_mag=0.0,
                        y_mag=0.0,
                        z_mag=0.0,
                        op_mode=0,
                        lin_acc_switch=0,
                        turn_switch=0
                    )
                    points.append(point)
                    packet_num += 1
                    
                except struct.error as e:
                    print(f"Warning: Error unpacking packet at offset {offset}: {e}")
                
                offset += packet_size
            
            print(f"Successfully parsed {len(points)} packets from A2 binary file")
            
        except FileNotFoundError:
            print(f"File not found: {binary_file_path}")
            return []
        except Exception as e:
            print(f"Error parsing A2 binary file: {e}")
            return []
        
        self.points = points
        return points

    def to_csv(self, output_file: str) -> bool:
        """Export parsed IMU points to CSV format for verification.
        
        Exports all parsed data points with angles in degrees and rates in deg/s.
        """
        if not self.points:
            print("No points to export")
            return False
        
        try:
            with open(output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                
                # Write header
                writer.writerow([
                    'timeITOW (msec)', 'time (s)', 'roll (deg)', 'pitch (deg)', 'heading (deg)',
                    'xRate (rad/s)', 'yRate (deg/s)', 'zRate (deg/s)',
                    'xAccel (m/s^2)', 'yAccel (m/s^2)', 'zAccel (m/s^2)'
                ])
                
                # Write data rows
                for point in self.points:
                    writer.writerow([
                        point.time_counter,
                        f"{point.time:.2f}",
                        f"{math.degrees(point.roll):.4f}",
                        f"{math.degrees(point.pitch):.4f}",
                        f"{math.degrees(point.yaw):.4f}",
                        f"{point.x_rate:.4f}",
                        f"{math.degrees(point.y_rate):.4f}",
                        f"{math.degrees(point.z_rate):.4f}",
                        f"{point.x_accel:.4f}",
                        f"{point.y_accel:.4f}",
                        f"{point.z_accel:.4f}"
                    ])
            
            print(f"Successfully exported {len(self.points)} points to: {output_file}")
            return True
            
        except Exception as e:
            print(f"Error exporting to CSV: {e}")
            return False
        #Convert Euler to radians.
        if not HAS_ROS:
            return None

        # Half angles
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(x=x, y=y, z=z, w=w)
    def _quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles (radians) to quaternion."""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(x=x, y=y, z=z, w=w)
    def to_imu_message(self, point: IMUPoint, frame_id: str = "imu") -> Imu:
        if not HAS_ROS:
            print("ROS not installed. Cannot create Imu messages.")
            return None

        msg = Imu()
        msg.header = Header()
        msg.header.stamp.sec = int(point.time)
        msg.header.stamp.nanosec = int((point.time % 1) * 1e9)
        msg.header.frame_id = frame_id

        # Orientation from Euler angles (roll, pitch, yaw)
        msg.orientation = self._quaternion_from_euler(
            point.roll,
            point.pitch,
            point.yaw
        )
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]  # Estimated covariance

        # Angular velocity (rad/s) - convert from radians if needed
        msg.angular_velocity = Vector3(
            x=point.x_rate,
            y=point.y_rate,
            z=point.z_rate
        )

        # Covariance for angular velocity (row-major about x, y, z axes)
        msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        # Linear acceleration (m/s^2)
        msg.linear_acceleration = Vector3(
            x=point.x_accel,
            y=point.y_accel,
            z=point.z_accel
        )
        # Covariance for linear acceleration (row-major x, y, z)
        msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        return msg

    def to_bag(self, output_path: str, topic_name: str = "/imu/data") -> bool:
        if not HAS_ROS:
            print("ROS not installed. Cannot create bag files.")
            return False

        points = self.points

        if not points:
            print("No points to write")
            return False

        try:
            writer = rosbag2_py.SequentialWriter()

            storage_options = rosbag2_py.StorageOptions(
                uri=output_path,
                storage_id='sqlite3'
            )

            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            writer.open(storage_options, converter_options)

            topic_info = rosbag2_py.TopicMetadata(
                id=0,
                name=topic_name,
                type='sensor_msgs/msg/Imu',
                serialization_format='cdr'
            )

            writer.create_topic(topic_info)

            for point in points:
                msg = self.to_imu_message(point)
                if msg:
                    timestamp_ns = int(point.time * 1e9)
                    writer.write(
                        topic_name,
                        serialize_message(msg),
                        timestamp_ns
                    )

            print(f"ROS2 bag file created: {output_path}")
            return True

        except Exception as e:
            print(f"Error creating ROS2 bag file: {e}")
            return False

    def inspect_bag(self, bag_path: str) -> None:
        print("ROS2 bag inspection not implemented (rosbag2_py has no simple reader API).")
        print(f"Bag path: {bag_path}")
