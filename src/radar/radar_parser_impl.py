import numpy as np
import struct
from pathlib import Path
from typing import List, Optional

try:
    import rosbag
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    import sensor_msgs.point_cloud2 as pc2
    import rospy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    PointCloud2 = None
    Header = None
    pc2 = None
    rospy = None

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
        points = []
        for det in points:
            r = det.range
            angle_rad = np.radians(det.angle)
            elev_rad = np.radians(det.elev)

            x = r * np.cos(angle_rad) * np.cos(elev_rad)
            y = r * np.sin(angle_rad) * np.cos(elev_rad)
            z = r * np.sin(elev_rad)
            intensity = det.snr

            points.append([x, y, z, intensity])

        points_array = np.array(points, dtype=np.float32)

        msg = pc2.create_cloud(
            Header(frame_id=frame_id),
            [('x', np.float32), ('y', np.float32),
             ('z', np.float32), ('intensity', np.float32)],
            points_array
        )

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
            with rosbag.Bag(output_path, 'w') as bag:
                # Group by frame
                frames = {}
                for point in points:
                    if point.frame not in frames:
                        frames[point.frame] = []
                    frames[point.frame].append(point)

                for frame_num in sorted(frames.keys()):
                    frame_points = frames[frame_num]

                    # Create timestamp
                    timestamp_ms = frame_points[0].timestamp
                    t = rospy.Time(secs=timestamp_ms // 1000,
                                 nsecs=(timestamp_ms % 1000) * 1000000)

                    # Convert to point cloud
                    msg = self.to_point_cloud(frame_points)
                    if msg:
                        bag.write(topic_name, msg, t)

            print(f" Bag file created: {output_path}")
            return True

        except Exception as e:
            print(f" Error creating bag file: {e}")
            return False

    def inspect_bag(self, bag_path: str) -> None:

        if not Path(bag_path).exists():
            print(f"Bag file not found: {bag_path}")
            return

        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                print(f"Bag file: {bag_path}")
                print(f" Duration: {bag.get_end_time() - bag.get_start_time():.2f}s")

                info = bag.get_type_and_message_count()
                print(f"  Topics: {len(info)}")
                for topic, (msg_type, msg_count) in info.items():
                    print(f"    - {topic}: {msg_count} messages ({msg_type})")

                # Show first message
                for topic, msg, t in bag.read_messages(limit=1):
                    print(f"\n  First message ({topic}) at {t.to_sec():.3f}s:")
                    if hasattr(msg, 'width') and hasattr(msg, 'height'):
                        print(f"    Points: {msg.width * msg.height}")

        except Exception as e:
            print(f"Error inspecting bag: {e}")
