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
        self.imu_fields = [ 'time_counter', 'time', 'qx', 'qy', 'qz', 'qw', 'x_accel', 'y_accel', 'z_accel','x_rate', 'y_rate', 'z_rate']
    
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

    def _quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles (radians) to quaternion."""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def to_imu_message(self, point: IMUPoint, frame_id: str = "imu") -> Imu:

        msg = Imu()
        msg.header = Header()
        msg.header.stamp.sec = int(point.time)
        msg.header.stamp.nanosec = int((point.time % 1) * 1e9)
        msg.header.frame_id = frame_id

        # Orientation from stored quaternion
        msg.orientation = Quaternion(x=point.qx, y=point.qy, z=point.qz, w=point.qw)
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        # Angular velocity (rad/s) - convert from radians if needed
        msg.angular_velocity = Vector3(
            x=point.x_rate,
            y=point.y_rate,
            z=point.z_rate
        )

        # Covariance for angular velocity (row-major about x, y, z axes) - ROS uses this weight in filtering 
        msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        # Linear acceleration (m/s^2)
        msg.linear_acceleration = Vector3(x=point.x_accel,y=point.y_accel,z=point.z_accel)

        # Covariance for linear acceleration (row-major x, y, z) - ROS uses this weight in filtering 
        msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        return msg

    def to_bag(self, output_path: str, topic_name: str = "/imu/data", serial_port=None) -> bool:

        try:

            #inital creation of bag
            writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py.StorageOptions(uri=output_path,storage_id='sqlite3')
            converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
            writer.open(storage_options, converter_options)
            topic_info = rosbag2_py.TopicMetadata(id=0,name=topic_name,type='sensor_msgs/msg/Imu',serialization_format='cdr')
            writer.create_topic(topic_info) 

            # Stream from serial port continuously
            if serial_port:                 
                packet_count = 0
                header = b'UUa20'
                packet_size = 55
                buffer = bytearray()
        
                try:
                    while True:
                        byte = serial_port.read(1) #read a byte 
                        if not byte:
                            continue
                        
                        buffer.extend(byte)
                        
                        # Look for header
                        if buffer[0:5] != header: #on loops worst case O(1) (find has worst O(n))
                            idx = buffer.find(header)
                            if idx > 0:
                                buffer = buffer[idx:]
                        
                        # Check if we have complete packet - if so extract and remove from buffer 
                        if len(buffer) >= packet_size:
                            packet = bytes(buffer[:packet_size])
                            buffer = buffer[packet_size:]
                            
                            try:
                                if packet[0:5] == header:
                                    payload = packet[5:53]
                                    if len(payload) >= 48:
                                        values = struct.unpack('<12f', payload[:48])
                                        
                                        # Extract Euler angles and convert to quaternion 
                                        roll_rad = math.radians(values[2])
                                        pitch_rad = math.radians(values[3])
                                        yaw_rad = math.radians(values[5])
                                        qx, qy, qz, qw = self._quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
                                        
                                        point = IMUPoint(
                                            time_counter=int(values[0]),
                                            time=values[0] / 1000.0,
                                            qx=qx,
                                            qy=qy,
                                            qz=qz,
                                            qw=qw,
                                            x_accel=values[9],
                                            y_accel=values[10],
                                            z_accel=values[11],
                                            x_rate=values[6],
                                            y_rate=values[7],
                                            z_rate=values[8]
                                        )
                                        
                                        msg = self.to_imu_message(point)
                                        if msg:
                                            timestamp_ns = int(point.time * 1e9)
                                            writer.write(topic_name, serialize_message(msg), timestamp_ns)
                                            packet_count += 1
                                            
                                            #DEBUG every 100 packets 
                                            if packet_count % 100 == 0:
                                                print(f"Recorded {packet_count} packets...")
                            except struct.error:
                                pass
                
                except KeyboardInterrupt:
                    print("\nStopped by user")
                print(f"Total packets recorded: {packet_count}")
            print(f"ROS2 bag file created: {output_path}")
            return True

        except Exception as e:
            print(f"Error creating ROS2 bag file: {e}")
            return False

    def inspect_bag(self, bag_path: str) -> None:
        print("ROS2 bag inspection not implemented (rosbag2_py has no simple reader API).")
        print(f"Bag path: {bag_path}")
