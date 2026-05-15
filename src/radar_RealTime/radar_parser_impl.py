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

    # TI radar serial packet constants
    # Header layout (40 bytes after magic):
    #   [magic 8B][version 4B][total_pkt_len 4B][platform 4B][frame_num 4B]
    #   [cpu_cycles 4B][num_obj 4B][num_tlv 4B][subframe_num 4B]
    MAGIC         = bytes.fromhex('0201040306050807')
    HEADER_LEN    = 40   # full header including magic
    PKT_LEN_OFF   = 12   # offset of total_packet_length within header (from magic start)
    FRAME_NUM_OFF = 20
    TIMESTAMP_OFF = 24
    NUM_TLV_OFF   = 32

    def _sync_to_magic(self, ser) -> bool:
        """Scan serial stream byte-by-byte until the 8-byte TI magic word is found."""
        buf = bytearray()
        b = ser.read(1)
        if not b:
            return False
        buf += b
        while len(buf) < 8:
            if not buf[-1:] == self.MAGIC[:len(buf)][-1:]:
                # mismatch — slide forward
                buf = bytearray()
                b = ser.read(1)
                if not b:
                    return False
                buf += b
                continue
            b = ser.read(1)
            if not b:
                return False
            buf += b
        return buf == self.MAGIC

    def _read_radar_packet(self, ser):
        """Read one full TI radar packet from serial (magic already consumed).
        Returns a list of RadarPoint or None on error."""
        # Read remainder of header (HEADER_LEN - 8 magic bytes already consumed)
        hdr_rest = ser.read(self.HEADER_LEN - 8)
        if len(hdr_rest) < self.HEADER_LEN - 8:
            return None

        # Reconstruct full header for parsing
        header = self.MAGIC + hdr_rest

        total_len   = u32(header, self.PKT_LEN_OFF)
        frame_num   = u32(header, self.FRAME_NUM_OFF)
        timestamp   = u32(header, self.TIMESTAMP_OFF)
        num_tlv     = u32(header, self.NUM_TLV_OFF)

        if total_len < self.HEADER_LEN or total_len > 65536:
            return None  # sanity check

        # Read the payload (everything after the header)
        payload_len = total_len - self.HEADER_LEN
        payload = ser.read(payload_len)
        if len(payload) < payload_len:
            return None

        # Reconstruct full packet for existing TLV parser
        packet = header + payload
        points = []
        idx = self.HEADER_LEN

        for _ in range(num_tlv):
            if idx + 8 > len(packet):
                break
            tlv_type   = u32(packet, idx)
            tlv_length = u32(packet, idx + 4)
            tlv_end    = idx + 8 + tlv_length

            if tlv_length < 8 or tlv_end > len(packet):
                break
            idx += 8

            if tlv_type == 1:
                count = tlv_length // 16
                for k in range(count):
                    r = f32(packet, idx)
                    a = f32(packet, idx + 4)
                    e = f32(packet, idx + 8)
                    d = f32(packet, idx + 12)
                    points.append(RadarPoint(
                        frame=frame_num,
                        timestamp=timestamp,
                        range=r,
                        angle=a,
                        elev=e,
                        doppler=d,
                        snr=0,
                        noise=0,
                    ))
                    idx += 16
            idx = tlv_end

        return points

    def parse_serial(self, radar_port: str, radar_baud: int,
                     duration: float = None, timeout: float = 1.0) -> List[RadarPoint]:
        """Stream TI radar packets from a live serial port.

        Args:
            radar_port: Serial port, e.g. '/dev/ttyUSB1'
            radar_baud: Baud rate, e.g. 921600
            duration:   Stop after this many seconds (None = until Ctrl-C).
            timeout:    Per-read serial timeout in seconds.
        """
        import serial as _serial
        import time as _time

        points   = []
        errors   = 0
        frames   = 0
        deadline = _time.monotonic() + duration if duration is not None else None

        try:
            with _serial.Serial(radar_port, radar_baud, timeout=timeout) as ser:
                ser.reset_input_buffer()
                print(f"Streaming radar from {radar_port} at {radar_baud} baud"
                      + (f" for {duration:.0f}s ..." if duration else " (Ctrl-C to stop) ..."))

                while True:
                    if deadline is not None and _time.monotonic() >= deadline:
                        print(f"\nRadar capture complete ({duration:.0f}s).")
                        break

                    if not self._sync_to_magic(ser):
                        errors += 1
                        continue

                    pkt_points = self._read_radar_packet(ser)
                    if pkt_points is None:
                        errors += 1
                        continue

                    points.extend(pkt_points)
                    frames += 1

        except KeyboardInterrupt:
            print("\nStopped by user.")
        except Exception as e:
            print(f"Error reading radar serial port: {e}")
            return []

        print(f"Captured {frames:,} frames | {len(points):,} points | Errors: {errors:,}")
        self.points = points
        return points

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
                id=0,
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
