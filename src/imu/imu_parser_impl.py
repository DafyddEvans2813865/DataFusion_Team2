import struct
import math
import numpy as np
from typing import List
import threading
import queue

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


class IMUParserMultithread:
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

def to_bag_multithreaded(self, output_path, topic_name, serial_port):

    raw_packet_queue = queue.Queue(maxsize=1000)
    msg_queue = queue.Queue(maxsize=500)
    stop_event = threading.Event()

    #Creating and starting threads
    threads_list = []
    threads_list.append(threading.Thread(target=self._reader_worker,args=(serial_port, raw_packet_queue, stop_event),daemon=False))
    threads_list.append(threading.Thread(target=self._parser_worker,args=(raw_packet_queue, msg_queue, stop_event),daemon=False))
    threads_list.append(threading.Thread(target=self._writer_worker,args=(msg_queue,output_path,topic_name,stop_event),daemon=False))

    for thread in threads_list:
        thread.start()
    try:
        for thread in threads_list:
            thread.join()
    except KeyboardInterrupt:
        print("Stopping...")
        stop_event.set()
        for thread in threads_list:
            thread.join()

    return True

def _reader_worker(self, serial_port, packet_queue, stop_event):
    buffer = bytearray()
    header = b'UUa20'
    packet_size = 55

    while not stop_event.is_set():
        # Read 1 byte
        byte = serial_port.read(1)
        if not byte:
            continue

        # Add to buffer
        buffer.extend(byte)

        # Sync to header (if not already synced)
        if buffer[0:5] != header:
            idx = buffer.find(header)
            if idx > 0:
                buffer = buffer[idx:]

        # Got packet?
        if len(buffer) >= packet_size:
            packet = bytes(buffer[:packet_size])
            buffer = buffer[packet_size:]

            # Put packet in queue
            try:
                packet_queue.put(packet, timeout=1)
            except queue.Full:
                print("Packet queue full!")

def _parser_worker(self, packet_queue, msg_queue, stop_event):
    while not stop_event.is_set():
        try:
            # Get packet from queue
            packet = packet_queue.get(timeout=1)

            # Extract payload (skip header)
            payload = packet[5:53]

            # Unpack payload (uint32 + double + 9 floats)
            seq, = struct.unpack_from('<I', payload, 0)
            t, = struct.unpack_from('<d', payload, 4)
            vals = struct.unpack_from('<9f', payload, 12)

            # Reorganize [seq, t, roll, pitch, yaw, xRate, yRate, zRate, xAccel, yAccel, zAccel]
            values = (seq, t) + vals

            # Extract Euler angles from values
            roll_rad = math.radians(values[2])
            pitch_rad = math.radians(values[3])
            yaw_rad = math.radians(values[5])

            # Convert to quaternion
            qx, qy, qz, qw = self._quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

            # Create IMUPoint
            point = IMUPoint(
                time_counter=int(values[0]),
                time=values[0] / 1000.0,
                qx=qx, qy=qy, qz=qz, qw=qw,
                x_accel=values[9],
                y_accel=values[10],
                z_accel=values[11],
                x_rate=values[6],
                y_rate=values[7],
                z_rate=values[8]
            )

            # Convert to ROS message
            msg = self.to_imu_message(point)
            timestamp_ns = int(point.time * 1e9)
            serialized = serialize_message(msg)

            # Put into writer queue
            msg_queue.put((serialized, timestamp_ns), timeout=1)

        except queue.Empty:
            continue
        except struct.error:
            pass

def _writer_worker(self, msg_queue, output_path, topic_name, stop_event):
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=output_path,storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    topic_info = rosbag2_py.TopicMetadata(id=0,name=topic_name,type='sensor_msgs/msg/Imu',serialization_format='cdr')
    writer.create_topic(topic_info)  

    packet_count = 0

    while not stop_event.is_set():
        try:
            serialized, timestamp_ns = msg_queue.get(timeout=1)
            writer.write(topic_name, serialized, timestamp_ns)
            packet_count += 1

            #log every 100 
            if packet_count % 100 == 0:
                print(f"Recorded {packet_count} packets...")

        except queue.Empty:
            continue

    print(f"Total packets recorded: {packet_count}")