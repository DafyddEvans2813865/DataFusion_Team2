import numpy as np
import struct
from pathlib import Path
from typing import List, Optional

try:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    import rclpy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    PointCloud2 = None
    PointField = None
    Header = None
    rclpy = None

from .radar_point import RadarPoint


def u32(data, i):
    """Extract 32-bit unsigned integer from bytes."""
    return int.from_bytes(data[i:i+4], 'little')


def f32(data, i):
    """Extract 32-bit float from bytes."""
    return struct.unpack('<f', data[i:i+4])[0]


class RadarParser:
    def __init__(self):
        self.points: List[RadarPoint] = []
        # Magic word for TI radar frame synchronization
        self.magic = bytes.fromhex('0201040306050807')

    def parse_hex_text(self, hex_text_path: str) -> List[RadarPoint]:
        points = []
        
        try:
            with open(hex_text_path, 'r') as f:
                hex_data = f.read().strip()
            raw_bytes = bytes.fromhex(hex_data.replace(' ', ''))

            buffer = bytearray(raw_bytes)
            
            while buffer:
                # Find magic word
                start = buffer.find(self.magic)
                if start == -1:
                    break
                
                # Need at least 16 bytes for header (8 bytes magic + 8 bytes header)
                if len(buffer) < start + 16:
                    break
                
                # Extract packet length
                pkt_len = int.from_bytes(buffer[start+12:start+16], 'little')
                if len(buffer) < start + pkt_len:
                    break
                
                # Extract packet
                packet = buffer[start:start+pkt_len]
                
                # Parse frame header
                frame_number = u32(packet, 20)
                timestamp = u32(packet, 24)
                num_tlv = u32(packet, 32)

                # Parse TLVs
                idx = 44
                
                for tlv_idx in range(num_tlv):
                    if idx + 8 > len(packet):
                        break
                    
                    tlv_type = u32(packet, idx)
                    tlv_length = u32(packet, idx + 4)
                    
                    tlv_end = idx + 8 + tlv_length
                    
                    if tlv_length < 8 or tlv_end > len(packet):
                        break
                    
                    idx += 8
                    
                    # TLV Type 1: Dynamic Object Detection Points
                    if tlv_type == 1:
                        count = tlv_length // 16
                        for k in range(count):
                            r = f32(packet, idx)
                            a = f32(packet, idx + 4)
                            e = f32(packet, idx + 8)
                            d = f32(packet, idx + 12)
                            
                            point = RadarPoint(
                                frame=frame_number,
                                timestamp=timestamp,
                                range=r,
                                angle=a,
                                elev=e,
                                doppler=d,
                                snr=0,
                                noise=0
                            )
                            points.append(point)
                            idx += 16
                        idx = tlv_end
                    else:
                        idx = tlv_end
                # Move to next packet
                buffer = buffer[start + pkt_len:]

        except FileNotFoundError:
            print(f" File not found: {hex_text_path}")
            return []
        except ValueError as e:
            print(f"Error parsing hex data: {e}")
            return []

        self.points = points
        return points

    def to_point_cloud(self, points: List[RadarPoint], frame_id: str = "radar") -> PointCloud2:
        if not HAS_ROS:
            print(" ROS not installed. Cannot create PointCloud2 messages.")
            return None

        if not points:
            return None

        # Convert spherical to Cartesian coordinates
        points_out = []
        for det in points:
            r = det.range
            angle_rad = np.radians(det.angle)
            elev_rad = np.radians(det.elev)

            x = r * np.cos(angle_rad) * np.cos(elev_rad)
            y = r * np.sin(angle_rad) * np.cos(elev_rad)
            z = r * np.sin(elev_rad)
            intensity = det.snr

            points_out.append([x, y, z, intensity])

        points_array = np.array(points_out, dtype=np.float32)

        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = frame_id

        # Set timestamp to zero (will be overridden in bag write)
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.height = 1
        msg.width = len(points_array)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16  # 4 fields * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.data = points_array.tobytes()

        return msg

    def to_bag(self, output_path: str, topic_name: str = "/radar/points") -> bool:
        if not HAS_ROS:
            print(" ROS not installed. Cannot create bag files.")
            return False

        points = self.points

        if not points:
            print(" No points to write")
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
                name=topic_name,
                type='sensor_msgs/msg/PointCloud2',
                serialization_format='cdr'
            )

            writer.create_topic(topic_info)

            # Group by frame
            frames = {}
            for point in points:
                if point.frame not in frames:
                    frames[point.frame] = []
                frames[point.frame].append(point)

            for frame_num in sorted(frames.keys()):
                frame_points = frames[frame_num]

                msg = self.to_point_cloud(frame_points)
                if msg:
                    timestamp_ns = int(frame_points[0].timestamp * 1e6)  # ms -> ns
                    writer.write(
                        topic_name,
                        serialize_message(msg),
                        timestamp_ns
                    )

            print(f" ROS2 bag file created: {output_path}")
            return True

        except Exception as e:
            print(f" Error creating ROS2 bag file: {e}")
            return False

    def inspect_bag(self, bag_path: str) -> None:

        print("ROS2 bag inspection not implemented (rosbag2_py has no simple reader API).")
        print(f"Bag path: {bag_path}")
