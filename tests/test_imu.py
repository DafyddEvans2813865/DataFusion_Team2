import sys
import unittest
from pathlib import Path
import struct
import threading
import queue

# Add src to path for imu imports 
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from imu.imu_point import IMUPoint
from imu.imu_parser_impl import IMUParser

# Check if ROS2 is available
try:
    from sensor_msgs.msg import Imu
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


class TestIMUPoint(unittest.TestCase):
    def test_imu_point_creation(self):
        point = IMUPoint(
            time_counter=0,
            time=1000.0,
            qx=0.0,
            qy=0.0,
            qz=0.707,
            qw=0.707,
            x_accel=0.5,
            y_accel=0.3,
            z_accel=9.8,
            x_rate=1.0,
            y_rate=0.5,
            z_rate=0.2
        )
        
        self.assertEqual(point.time_counter, 0)
        self.assertEqual(point.time, 1000.0)
        self.assertEqual(point.qx, 0.0)
        self.assertEqual(point.qy, 0.0)
        self.assertEqual(point.x_accel, 0.5)

    def test_imu_point_to_dict(self):
        point = IMUPoint(
            time_counter=1,
            time=2000.0,
            qx=0.1,
            qy=0.2,
            qz=0.3,
            qw=0.9,
            x_accel=0.1,
            y_accel=0.2,
            z_accel=9.8,
            x_rate=0.5,
            y_rate=0.3,
            z_rate=0.1
        )
        
        point_dict = point.to_dict()
        
        self.assertIsInstance(point_dict, dict)
        self.assertEqual(point_dict['time_counter'], 1)
        self.assertEqual(point_dict['time'], 2000.0)
        self.assertEqual(point_dict['qx'], 0.1)
        self.assertEqual(point_dict['qy'], 0.2)


class TestIMUParser(unittest.TestCase):

    def setUp(self):
        self.parser = IMUParser()

    def test_parser_initialization(self):
        self.assertIsNotNone(self.parser)
        self.assertEqual(len(self.parser.points), 0)
        self.assertIsInstance(self.parser.points, list)

    def test_quaternion_conversion(self):
        """Test that Euler angles convert to quaternion correctly."""
        import math
        
        # Test identity rotation (0, 0, 0)
        qx, qy, qz, qw = self.parser._quaternion_from_euler(0, 0, 0)
        self.assertAlmostEqual(qx, 0.0, places=5)
        self.assertAlmostEqual(qy, 0.0, places=5)
        self.assertAlmostEqual(qz, 0.0, places=5)
        self.assertAlmostEqual(qw, 1.0, places=5)
        
        # Test 90 degree rotation around Z axis
        qx, qy, qz, qw = self.parser._quaternion_from_euler(0, 0, math.pi / 2)
        self.assertAlmostEqual(qx, 0.0, places=5)
        self.assertAlmostEqual(qy, 0.0, places=5)
        self.assertAlmostEqual(qz, 0.707, places=2)
        self.assertAlmostEqual(qw, 0.707, places=2)


class TestPayloadParsing(unittest.TestCase):
    """Test payload unpacking for multithreaded parser."""
    
    def test_payload_unpacking(self):
        """Test that payload unpacks correctly: uint32 + double + 9 floats."""
        # Create a mock 48-byte payload
        seq = 123
        t = 45.67
        vals = (10.0, 20.0, 30.0, 1.0, 2.0, 3.0, 9.8, 0.5, 0.2)
        
        payload = struct.pack('<I', seq) + struct.pack('<d', t) + struct.pack('<9f', *vals)
        
        self.assertEqual(len(payload), 48)
        
        # Unpack like the parser does
        seq_unpacked, = struct.unpack_from('<I', payload, 0)
        t_unpacked, = struct.unpack_from('<d', payload, 4)
        vals_unpacked = struct.unpack_from('<9f', payload, 12)
        values = (seq_unpacked, t_unpacked) + vals_unpacked
        
        # Verify
        self.assertEqual(values[0], seq)
        self.assertAlmostEqual(values[1], t, places=5)
        self.assertAlmostEqual(values[2], 10.0, places=5)
        self.assertAlmostEqual(values[10], 0.2, places=5)
    
    def test_packet_structure(self):
        """Test A2 packet structure: header + payload + checksum."""
        header = b'UUa20'
        payload = b'\x00' * 48
        
        packet = header + payload
        
        self.assertEqual(len(packet), 53)
        self.assertEqual(packet[0:5], header)
        self.assertEqual(packet[5:53], payload)


class TestMultithreadingStructure(unittest.TestCase):
    """Test multithreading architecture."""
    
    def test_queue_creation(self):
        """Test queue initialization."""
        raw_packet_queue = queue.Queue(maxsize=1000)
        msg_queue = queue.Queue(maxsize=500)
        
        # Queues start empty
        self.assertTrue(raw_packet_queue.empty())
        self.assertTrue(msg_queue.empty())
        self.assertEqual(raw_packet_queue.maxsize, 1000)
        self.assertEqual(msg_queue.maxsize, 500)
    
    def test_stop_event(self):
        """Test threading Event for graceful shutdown."""
        stop_event = threading.Event()
        
        # Should start as false
        self.assertFalse(stop_event.is_set())
        
        # Set it
        stop_event.set()
        self.assertTrue(stop_event.is_set())
        
        # Clear it
        stop_event.clear()
        self.assertFalse(stop_event.is_set())
    
    def test_queue_put_get(self):
        """Test queue thread-safe operations."""
        test_queue = queue.Queue(maxsize=10)
        test_packet = b'UUa20' + b'\x00' * 48
        
        # Put and get
        test_queue.put(test_packet, timeout=1)
        retrieved = test_queue.get(timeout=1)
        
        self.assertEqual(retrieved, test_packet)
        self.assertTrue(test_queue.empty())


class TestIMUMessageConversion(unittest.TestCase):
    """Test conversion to ROS2 message."""
    
    def setUp(self):
        self.parser = IMUParser()
    
    @unittest.skipIf(not HAS_ROS, "ROS2 not installed")
    def test_imu_message_creation(self):
        """Test that IMUPoint converts to ROS2 Imu message."""
        point = IMUPoint(
            time_counter=100,
            time=100.5,
            qx=0.0, qy=0.0, qz=0.707, qw=0.707,
            x_accel=1.0, y_accel=2.0, z_accel=9.8,
            x_rate=0.1, y_rate=0.2, z_rate=0.3
        )
        
        msg = self.parser.to_imu_message(point, frame_id="imu")
        
        self.assertIsNotNone(msg)
        self.assertEqual(msg.header.frame_id, "imu")
        self.assertAlmostEqual(msg.orientation.z, 0.707, places=2)
        self.assertAlmostEqual(msg.linear_acceleration.x, 1.0, places=5)


if __name__ == '__main__':
    unittest.main()
