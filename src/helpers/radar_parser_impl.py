import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import rospy
import struct
from pathlib import Path
from typing import List, Optional

from radar_detection import RadarDetection


class RadarParser:
    def __init__(self):
        self.detections: List[RadarDetection] = []

    def parse_hex_text(self, hex_text_path: str) -> List[RadarDetection]:
        detections = []
        try:
            with open(hex_text_path, 'r') as f:
                hex_data = f.read().strip()

            # Convert hex string to bytes
            raw_bytes = bytes.fromhex(hex_data.replace(' ', ''))

            # Parse the binary data
            idx = 0
            while idx + 20 <= len(raw_bytes):
                frame_num = struct.unpack('I', raw_bytes[idx:idx+4])[0]
                timestamp = struct.unpack('I', raw_bytes[idx+4:idx+8])[0]
                range_val = struct.unpack('f', raw_bytes[idx+8:idx+12])[0]
                angle = struct.unpack('f', raw_bytes[idx+12:idx+16])[0]
                doppler = struct.unpack('f', raw_bytes[idx+16:idx+20])[0]

                detection = RadarDetection(
                    frame=frame_num,
                    timestamp=timestamp,
                    range=range_val,
                    angle=angle,
                    elev=0.0,
                    doppler=doppler,
                    snr=0,
                    noise=0
                )
                detections.append(detection)
                idx += 20

        except FileNotFoundError:
            print(f"✗ File not found: {hex_text_path}")
            return []
        except ValueError as e:
            print(f"✗ Error parsing hex data: {e}")
            return []

        self.detections = detections
        return detections



    def to_point_cloud(self, detections: List[RadarDetection], frame_id: str = "radar") -> PointCloud2:
        """
        Convert detections to ROS PointCloud2 message.
        """
        if not detections:
            return None

        # Convert spherical to Cartesian coordinates
        points = []
        for det in detections:
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

    def to_bag(self, output_path: str, detections: Optional[List[RadarDetection]] = None,
              topic_name: str = "/radar/points") -> bool:
        """
        Write detections to ROS bag file.

        """
        if detections is None:
            detections = self.detections

        if not detections:
            print("✗ No detections to write")
            return False

        try:
            with rosbag.Bag(output_path, 'w') as bag:
                # Group by frame
                frames = {}
                for detection in detections:
                    if detection.frame not in frames:
                        frames[detection.frame] = []
                    frames[detection.frame].append(detection)

                for frame_num in sorted(frames.keys()):
                    frame_detections = frames[frame_num]

                    # Create timestamp
                    timestamp_ms = frame_detections[0].timestamp
                    t = rospy.Time(secs=timestamp_ms // 1000,
                                 nsecs=(timestamp_ms % 1000) * 1000000)

                    # Convert to point cloud
                    msg = self.to_point_cloud(frame_detections)
                    if msg:
                        bag.write(topic_name, msg, t)

            return True

        except Exception as e:
            print(f"✗ Error creating bag file: {e}")
            return False

    def inspect_bag(self, bag_path: str) -> None:
        """
        Inspect contents of a bag file.

        Args:
            bag_path: Path to .bag file
        """

        if not Path(bag_path).exists():
            print(f"✗ Bag file not found: {bag_path}")
            return

        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                print(f"✓ Bag file: {bag_path}")
                print(f"  Duration: {bag.get_end_time() - bag.get_start_time():.2f}s")

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
            print(f"✗ Error inspecting bag: {e}")
