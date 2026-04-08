import struct
import numpy as np
from pathlib import Path
from typing import List, Optional

from .imu_point import IMUPoint

try:
    import rosbag
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Quaternion, Vector3
    from std_msgs.msg import Header
    import rospy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Imu = None
    Quaternion = None
    Vector3 = None
    Header = None
    rospy = None


class IMUParser:
    """Parser for IMU binary data with 0x5555 packet headers."""
    
    # IMU packet structure: [0x55 0x55][type(2)][length(1)][payload][checksum(2)]
    IMU_FMT = '<IdfffffffffffffffBBB'
    IMU_HEADER = b'\x55\x55'
    
    def __init__(self):
        self.points: List[IMUPoint] = []
        self.imu_size = struct.calcsize(self.IMU_FMT)
        self.imu_fields = [
            'time_counter', 'time',
            'roll', 'pitch', 'heading',
            'x_accel', 'y_accel', 'z_accel',
            'x_rate', 'y_rate', 'z_rate',
            'x_rate_bias', 'y_rate_bias', 'z_rate_bias',
            'x_mag', 'y_mag', 'z_mag',
            'op_mode', 'lin_acc_switch', 'turn_switch'
        ]
    
    def parse_binary_file(self, binary_file_path: str) -> List[IMUPoint]:
        """Parse binary IMU data file and extract IMU data points."""
        points = []
        
        try:
            with open(binary_file_path, 'rb') as f:
                raw_bytes = f.read()
            
            buffer = bytearray(raw_bytes)
            
            while buffer:
                # Find header
                start = buffer.find(self.IMU_HEADER)
                if start == -1:
                    break
                
                # Need at least 5 bytes: header(2) + type(2) + length(1)
                if len(buffer) < start + 5:
                    break
                
                # Align buffer to header
                if start > 0:
                    buffer = buffer[start:]
                
                if len(buffer) < 5:
                    break
                
                # Extract length at offset 4
                length = buffer[4]
                
                # Full packet: header(2) + type(2) + length(1) + payload(length) + checksum(2)
                total_len = 5 + length + 2
                
                # Ensure we have full packet
                if len(buffer) < total_len:
                    break
                
                # Extract payload
                payload = buffer[5:5+self.imu_size]
                
                if len(payload) < self.imu_size:
                    buffer = buffer[1:]
                    continue
                
                try:
                    values = struct.unpack(self.IMU_FMT, payload[:self.imu_size])
                    
                    # Create IMUPoint from parsed values
                    point = IMUPoint(
                        time_counter=values[0],
                        time=values[1],
                        roll=values[2],
                        pitch=values[3],
                        heading=values[4],
                        x_accel=values[5],
                        y_accel=values[6],
                        z_accel=values[7],
                        x_rate=values[8],
                        y_rate=values[9],
                        z_rate=values[10],
                        x_rate_bias=values[11],
                        y_rate_bias=values[12],
                        z_rate_bias=values[13],
                        x_mag=values[14],
                        y_mag=values[15],
                        z_mag=values[16],
                        op_mode=values[17],
                        lin_acc_switch=values[18],
                        turn_switch=values[19]
                    )
                    points.append(point)
                
                except struct.error as e:
                    print(f"IMU parse error: {e}")
                
                # Move to next packet
                buffer = buffer[total_len:]
        
        
        except FileNotFoundError:
            print(f"File not found: {binary_file_path}")
            return []
        except Exception as e:
            print(f"Error parsing IMU file: {e}")
            return []
        
        self.points = points
        return points

    def _quaternion_from_euler(self, roll: float, pitch: float, yaw: float) -> Quaternion:
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

    def to_imu_message(self, point: IMUPoint, frame_id: str = "imu") -> Imu:
        #Convert IMUPoint to ROS Imu message
        if not HAS_ROS:
            print("ROS not installed. Cannot create Imu messages.")
            return None

        timestamp_ms = int(point.time * 1000) 
        t = rospy.Time(secs=timestamp_ms // 1000, nsecs=(timestamp_ms % 1000) * 1000000)

        msg = Imu()
        msg.header = Header(stamp=t, frame_id=frame_id)

        # Orientation (Not provided by this IMU? ) 
        msg.orientation = Quaternion(x=0, y=0, z=0, w=1)
        msg.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]  # Unknown

        # Angular velocity (rad/s)
        msg.angular_velocity = Vector3(
            x=point.x_rate,
            y=point.y_rate,
            z=point.z_rate
        )

        # Covariance for angular velocity (row-major about x, y, z axes)
        msg.angular_velocity_covariance = [0] * 9

        # Linear acceleration (m/s^2)
        msg.linear_acceleration = Vector3(
            x=point.x_accel,
            y=point.y_accel,
            z=point.z_accel
        )
        # Covariance for linear acceleration (row-major x, y, z)
        msg.linear_acceleration_covariance = [0] * 9

        return msg

    def to_bag(self, output_path: str, topic_name: str = "/imu/data") -> bool:
        """
        Create a ROS bag file from parsed IMU data.
        """
        if not HAS_ROS:
            print("ROS not installed. Cannot create bag files.")
            return False

        points = self.points

        if not points:
            print("No points to write")
            return False

        try:
            with rosbag.Bag(output_path, 'w') as bag:
                for point in points:
                    msg = self.to_imu_message(point)
                    if msg:
                        # Use time from point for the bag timestamp
                        timestamp_ms = int(point.time * 1000)
                        t = rospy.Time(secs=timestamp_ms // 1000,
                                     nsecs=(timestamp_ms % 1000) * 1000000)
                        bag.write(topic_name, msg, t)

            print(f"Bag file created: {output_path}")
            return True

        except Exception as e:
            print(f"Error creating bag file: {e}")
            return False

    def inspect_bag(self, bag_path: str) -> None:
        if not HAS_ROS:
            print("ROS not installed. Cannot inspect bag files.")
            return

        if not Path(bag_path).exists():
            print(f"Bag file not found: {bag_path}")
            return

        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                print(f"Bag file: {bag_path}")
                print(f"  Duration: {bag.get_end_time() - bag.get_start_time():.2f} seconds")
                print(f"  Start time: {bag.get_start_time()}")
                print(f"  End time: {bag.get_end_time()}")

                # Get topic information
                info = bag.get_type_and_topic_info()
                print(f"  Topics:")
                for topic, topic_info in info.topics.items():
                    print(f"    {topic}: {topic_info.msg_type} ({topic_info.message_count} messages)")

        except Exception as e:
            print(f"Error inspecting bag file: {e}")

