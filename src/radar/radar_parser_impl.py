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
 
 
def u16(data, i):
    """Extract 16-bit unsigned integer from bytes."""
    return int.from_bytes(data[i:i+2], 'little')
 
 
def u32(data, i):
    """Extract 32-bit unsigned integer from bytes."""
    return int.from_bytes(data[i:i+4], 'little')
 
 
def f32(data, i):
    """Extract 32-bit float from bytes."""
    return struct.unpack('<f', data[i:i+4])[0]
 
 
# Frame header field offsets (all relative to start of magic word)
# Total header length: 40 bytes
# Offset  0: Magic Word       (8 bytes, uint16_t[4])
# Offset  8: Version          (4 bytes, uint32_t)
# Offset 12: Total Pkt Length (4 bytes, uint32_t)
# Offset 16: Platform         (4 bytes, uint32_t)
# Offset 20: Frame Number     (4 bytes, uint32_t)
# Offset 24: Time [CPU cycles](4 bytes, uint32_t)
# Offset 28: Num Detected Obj (4 bytes, uint32_t)
# Offset 32: Num TLVs         (4 bytes, uint32_t)
# Offset 36: Subframe Number  (4 bytes, uint32_t)
FRAME_HEADER_SIZE = 40
 
# TLV type identifiers
TLV_DETECTED_POINTS       = 1     # Cartesian: X, Y, Z, doppler (each float32)
TLV_SIDE_INFO             = 7     # SNR + noise per point (each uint16)
TLV_SPHERICAL_POINTS      = 1000  # Spherical: range, azimuth(rad), elev(rad), doppler
TLV_TARGET_LIST           = 1010
TLV_TARGET_INDEX          = 1011
TLV_TARGET_HEIGHT         = 1012
TLV_COMPRESSED_POINTS     = 1020
TLV_PRESENCE              = 1021
 
 
class RadarParser:
    def __init__(self):
        self.points: List[RadarPoint] = []
        # Magic word for TI radar frame synchronization
        # Initialised as {0x0102, 0x0304, 0x0506, 0x0708} stored little-endian
        self.magic = bytes.fromhex('0201040306050807')
 
    def _parse_packet(self, packet: bytes) -> List[RadarPoint]:
        """
        Parse a single TI mmWave UART frame packet and return RadarPoint list.
 
        Frame header layout (40 bytes):
          [0:8]   Magic Word
          [8:12]  Version
          [12:16] Total Packet Length
          [16:20] Platform
          [20:24] Frame Number
          [24:28] Time (CPU cycles)
          [28:32] Num Detected Obj
          [32:36] Num TLVs
          [36:40] Subframe Number
 
        TLV layout (8-byte header + payload):
          [0:4]  Type  (uint32)
          [4:8]  Length of payload in bytes (uint32)
          [8:8+Length] Payload
 
        TLV Type 1 – Detected Points (Cartesian, 16 bytes each):
          X [m], Y [m], Z [m], Doppler [m/s]  (all float32)
 
        TLV Type 7 – Side Info (4 bytes per point):
          SNR [0.1 dB units] (uint16), Noise [0.1 dB units] (uint16)
 
        TLV Type 1000 – Spherical Points (16 bytes each):
          Range [m], Azimuth [rad], Elevation [rad], Doppler [m/s]  (all float32)
        """
        frame_number  = u32(packet, 20)
        timestamp     = u32(packet, 24)   # CPU cycles
        num_detected  = u32(packet, 28)
        num_tlv       = u32(packet, 32)
 
        # Accumulate points and side-info separately, then merge
        points: List[RadarPoint] = []
        side_info: List[tuple] = []  # (snr, noise) per point
 
        idx = FRAME_HEADER_SIZE  # TLVs start immediately after the 40-byte header
 
        for _ in range(num_tlv):
            tlv_type = u32(packet, idx)
            tlv_length = u32(packet, idx + 4)

            # TI TLV length includes the 8-byte TLV header
            if tlv_length < 8:
                break

            payload_start = idx + 8
            payload_length = tlv_length - 8
            payload_end = payload_start + payload_length

            if payload_end > len(packet):
                break
 
            # ----------------------------------------------------------------
            # TLV 1: Detected Points – Cartesian (X, Y, Z, Doppler)
            # ----------------------------------------------------------------
            if tlv_type == TLV_DETECTED_POINTS:
                bytes_per_point = 16  # 4 × float32
                count = payload_length // bytes_per_point
                p = payload_start
                for _ in range(count):
                    x       = f32(packet, p)
                    y       = f32(packet, p + 4)
                    z       = f32(packet, p + 8)
                    doppler = f32(packet, p + 12)
 
                    # Store Cartesian directly; convert to spherical for
                    # backward-compatible RadarPoint fields.
                    r     = float(np.sqrt(x**2 + y**2 + z**2))
                    az    = float(np.degrees(np.arctan2(y, x)))     # azimuth  [deg]
                    elev  = float(np.degrees(np.arctan2(z, np.sqrt(x**2 + y**2))))  # elev [deg]
 
                    points.append(RadarPoint(
                        frame=frame_number,
                        timestamp=timestamp,
                        range=r,
                        angle=az,
                        elev=elev,
                        doppler=doppler,
                        snr=0,
                        noise=0,
                    ))
                    p += bytes_per_point
 
            # ----------------------------------------------------------------
            # TLV 7: Side Info – SNR + Noise per detected point
            # ----------------------------------------------------------------
            elif tlv_type == TLV_SIDE_INFO:
                bytes_per_point = 4  # 2 × uint16
                count = payload_length // bytes_per_point
                p = payload_start
                for _ in range(count):
                    snr   = u16(packet, p)      # multiples of 0.1 dB
                    noise = u16(packet, p + 2)  # multiples of 0.1 dB
                    side_info.append((snr, noise))
                    p += bytes_per_point
 
            # ----------------------------------------------------------------
            # TLV 1000: Spherical Points – Range, Azimuth, Elevation, Doppler
            # ----------------------------------------------------------------
            elif tlv_type == TLV_SPHERICAL_POINTS:
                bytes_per_point = 16  # 4 × float32
                count = payload_length // bytes_per_point
                p = payload_start
                for _ in range(count):
                    r       = f32(packet, p)
                    az_rad  = f32(packet, p + 4)
                    el_rad  = f32(packet, p + 8)
                    doppler = f32(packet, p + 12)
 
                    points.append(RadarPoint(
                        frame=frame_number,
                        timestamp=timestamp,
                        range=r,
                        angle=float(np.degrees(az_rad)),
                        elev=float(np.degrees(el_rad)),
                        doppler=doppler,
                        snr=0,
                        noise=0,
                    ))
                    p += bytes_per_point

            # ----------------------------------------------------------------
            # TLV 1020: Compressed Points
            # ----------------------------------------------------------------
            elif tlv_type == TLV_COMPRESSED_POINTS:

                # Units header is 20 bytes
                if payload_length < 20:
                    idx += tlv_length
                    continue

                p = payload_start

                elev_unit = f32(packet, p)
                azimuth_unit = f32(packet, p + 4)
                doppler_unit = f32(packet, p + 8)
                range_unit = f32(packet, p + 12)
                snr_unit = f32(packet, p + 16)

                p += 20

                bytes_per_point = 8
                count = (payload_length - 20) // bytes_per_point

                for _ in range(count):
                    elevation = struct.unpack('<b', packet[p:p+1])[0]
                    azimuth = struct.unpack('<b', packet[p+1:p+2])[0]
                    doppler = struct.unpack('<h', packet[p+2:p+4])[0]
                    rng = u16(packet, p + 4)
                    snr = u16(packet, p + 6)

                    elev = elevation * elev_unit
                    angle = azimuth * azimuth_unit
                    range_m = rng * range_unit
                    doppler_ms = doppler * doppler_unit
                    snr_db = snr * snr_unit

                    points.append(RadarPoint(
                        frame=frame_number,
                        timestamp=timestamp,
                        range=range_m,
                        angle=angle,
                        elev=elev,
                        doppler=doppler_ms,
                        snr=snr_db,
                        noise=0,
                    ))

                    p += bytes_per_point
 
            idx += tlv_length

            # Some TI demos pad packets to 32-byte boundaries
            while idx % 32 != 0 and idx < len(packet):
                idx += 1
 
        # Merge side-info (TLV 7) into the points parsed from TLV 1 / 1000.
        # The spec guarantees side-info entries are in the same order as points.
        for i, (snr, noise) in enumerate(side_info):
            if i < len(points):
                points[i].snr   = snr
                points[i].noise = noise
 
        return points
 
    def parse_hex_text(self, hex_text_path: str) -> List[RadarPoint]:
        all_points: List[RadarPoint] = []
 
        try:
            with open(hex_text_path, 'r') as f:
                hex_data = f.read().strip()
            raw_bytes = bytes.fromhex(hex_data.replace(' ', ''))
        except FileNotFoundError:
            print(f"File not found: {hex_text_path}")
            return []
        except ValueError as e:
            print(f"Error parsing hex data: {e}")
            return []
 
        buffer = bytearray(raw_bytes)
 
        while buffer:
            start = buffer.find(self.magic)
            if start == -1:
                break
 
            try:
                pkt_len = u32(buffer, start + 12)
            except Exception:
                break

            # Sanity-check: packet length must be at least the header size
            if pkt_len <= FRAME_HEADER_SIZE:
                buffer = buffer[start + 8:]
                continue

            if len(buffer) < start + pkt_len:
                break
 
            packet = bytes(buffer[start:start + pkt_len])
            all_points.extend(self._parse_packet(packet))
 
            buffer = buffer[start + pkt_len:]
 
        self.points = all_points
        return all_points
 
    # ------------------------------------------------------------------
    # ROS / bag helpers (unchanged from original)
    # ------------------------------------------------------------------
 
    def to_point_cloud(self, points: List[RadarPoint], frame_id: str = "radar") -> "PointCloud2":
        if not HAS_ROS:
            print("ROS not installed. Cannot create PointCloud2 messages.")
            return None
 
        if not points:
            return None
 
        # Convert spherical to Cartesian for PointCloud2 output
        points_out = []
        for det in points:
            r          = det.range
            angle_rad  = np.radians(det.angle)
            elev_rad   = np.radians(det.elev)
 
            x = r * np.cos(angle_rad) * np.cos(elev_rad)
            y = r * np.sin(angle_rad) * np.cos(elev_rad)
            z = r * np.sin(elev_rad)
            intensity = det.snr * 0.1  # convert 0.1 dB units to dB
 
            points_out.append([x, y, z, intensity])
 
        points_array = np.array(points_out, dtype=np.float32)
 
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
 
        msg.height = 1
        msg.width = len(points_array)
 
        msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points_array.tobytes()
        return msg
 
    def to_bag(self, output_path: str, topic_name: str = "/radar/points") -> bool:
        if not HAS_ROS:
            print("ROS not installed. Cannot create bag files.")
            return False
 
        if not self.points:
            print("No points to write")
            return False
 
        try:
            writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id='sqlite3')
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            writer.open(storage_options, converter_options)
 
            topic_info = rosbag2_py.TopicMetadata(
                id=0,
                name=topic_name,
                type='sensor_msgs/msg/PointCloud2',
                serialization_format='cdr'
            )
            writer.create_topic(topic_info)
 
            frames = {}
            for point in self.points:
                frames.setdefault(point.frame, []).append(point)
 
            for frame_num in sorted(frames.keys()):
                frame_points = frames[frame_num]
                msg = self.to_point_cloud(frame_points)
                if msg:
                    # Timestamp stored as CPU cycles; treat as milliseconds → nanoseconds
                    timestamp_ns = int(frame_points[0].timestamp * 1e6)
                    writer.write(topic_name, serialize_message(msg), timestamp_ns)
 
            print(f"ROS2 bag file created: {output_path}")
            return True
 
        except Exception as e:
            print(f"Error creating ROS2 bag file: {e}")
            return False
 
    def inspect_bag(self, bag_path: str) -> None:
        print("ROS2 bag inspection not implemented (rosbag2_py has no simple reader API).")
        print(f"Bag path: {bag_path}")