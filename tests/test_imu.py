import sys
import unittest
import struct
from pathlib import Path

# Add src to path for imu imports 
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from imu.imu_point import IMUPoint
from imu.imu_parser_impl import IMUParser


def create_imu_packet(
    time_counter=0,
    time=1000.0,
    roll=0.0,
    pitch=0.0,
    heading=0.0,
    x_accel=0.0,
    y_accel=0.0,
    z_accel=0.0,
    x_rate=0.0,
    y_rate=0.0,
    z_rate=0.0,
    x_rate_bias=0.0,
    y_rate_bias=0.0,
    z_rate_bias=0.0,
    x_mag=0.0,
    y_mag=0.0,
    z_mag=0.0,
    op_mode=0,
    lin_acc_switch=0,
    turn_switch=0
):
    """Create an IMU packet with 0x5555 header."""
    # Pack IMU data
    imu_fmt = '<IdfffffffffffffffBBB'
    payload = struct.pack(
        imu_fmt,
        time_counter,
        time,
        roll, pitch, heading,
        x_accel, y_accel, z_accel,
        x_rate, y_rate, z_rate,
        x_rate_bias, y_rate_bias, z_rate_bias,
        x_mag, y_mag, z_mag,
        op_mode, lin_acc_switch, turn_switch
    )
    
    # IMU packet structure: [0x55 0x55][type(2)][length(1)][payload][checksum(2)]
    header = b'\x55\x55'
    packet_type = struct.pack('<H', 0)  # type (little-endian short)
    length = struct.pack('B', len(payload))  # length (1 byte)
    checksum = struct.pack('<H', 0)  # dummy checksum
    
    packet = header + packet_type + length + payload + checksum
    return packet


class TestIMUPoint(unittest.TestCase):
    def test_imu_point_creation(self):
        point = IMUPoint(
            time_counter=0,
            time=1000.0,
            roll=10.5,
            pitch=20.3,
            heading=45.0,
            x_accel=0.5,
            y_accel=0.3,
            z_accel=9.8,
            x_rate=1.0,
            y_rate=0.5,
            z_rate=0.2,
            x_rate_bias=0.01,
            y_rate_bias=0.01,
            z_rate_bias=0.01,
            x_mag=10.0,
            y_mag=20.0,
            z_mag=30.0,
            op_mode=1,
            lin_acc_switch=1,
            turn_switch=0
        )
        
        self.assertEqual(point.time_counter, 0)
        self.assertEqual(point.time, 1000.0)
        self.assertEqual(point.roll, 10.5)
        self.assertEqual(point.pitch, 20.3)
        self.assertEqual(point.x_accel, 0.5)

    def test_imu_point_to_dict(self):
        point = IMUPoint(
            time_counter=1,
            time=2000.0,
            roll=5.0,
            pitch=10.0,
            heading=20.0,
            x_accel=0.1,
            y_accel=0.2,
            z_accel=9.8,
            x_rate=0.5,
            y_rate=0.3,
            z_rate=0.1,
            x_rate_bias=0.01,
            y_rate_bias=0.01,
            z_rate_bias=0.01,
            x_mag=15.0,
            y_mag=25.0,
            z_mag=35.0,
            op_mode=2,
            lin_acc_switch=0,
            turn_switch=1
        )
        
        point_dict = point.to_dict()
        
        self.assertIsInstance(point_dict, dict)
        self.assertEqual(point_dict['time_counter'], 1)
        self.assertEqual(point_dict['time'], 2000.0)
        self.assertEqual(point_dict['roll'], 5.0)
        self.assertEqual(point_dict['pitch'], 10.0)


class TestIMUParser(unittest.TestCase):

    def setUp(self):
        self.parser = IMUParser()

    def test_parser_initialization(self):
        self.assertIsNotNone(self.parser)
        self.assertEqual(len(self.parser.points), 0)
        self.assertIsInstance(self.parser.points, list)

    def test_parse_binary_file_valid_data(self):
        """Test parsing a single IMU measurement."""
        test_packet = create_imu_packet(
            time_counter=0,
            time=1000.0,
            roll=10.5,
            pitch=20.3,
            heading=45.0,
            x_accel=0.5,
            y_accel=0.3,
            z_accel=9.8
        )
        
        test_file = Path(__file__).parent / "test_imu_data.bin"
        
        try:
            with open(test_file, 'wb') as f:
                f.write(test_packet)
            
            points = self.parser.parse_binary_file(str(test_file))
            
            self.assertEqual(len(points), 1)
            self.assertEqual(points[0].time_counter, 0)
            self.assertAlmostEqual(points[0].time, 1000.0, places=5)
            self.assertAlmostEqual(points[0].roll, 10.5, places=5)
            self.assertAlmostEqual(points[0].pitch, 20.3, places=5)
            self.assertAlmostEqual(points[0].heading, 45.0, places=5)
            self.assertAlmostEqual(points[0].x_accel, 0.5, places=5)
        finally:
            if test_file.exists():
                test_file.unlink()

    def test_parse_binary_file_multiple_points(self):
        """Test parsing multiple IMU measurements."""
        test_data = b''
        for i in range(3):
            packet = create_imu_packet(
                time_counter=i,
                time=1000.0 + i * 100,
                roll=10.0 + i,
                pitch=20.0 + i * 2,
                heading=45.0 + i * 5
            )
            test_data += packet
        
        test_file = Path(__file__).parent / "test_imu_data_multi.bin"
        
        try:
            with open(test_file, 'wb') as f:
                f.write(test_data)
            
            points = self.parser.parse_binary_file(str(test_file))
            
            self.assertEqual(len(points), 3)
            for i in range(3):
                self.assertEqual(points[i].time_counter, i)
                self.assertAlmostEqual(points[i].time, 1000.0 + i * 100, places=5)
                self.assertAlmostEqual(points[i].roll, 10.0 + i, places=5)
        finally:
            if test_file.exists():
                test_file.unlink()

    def test_parse_binary_file_file_not_found(self):
        points = self.parser.parse_binary_file("/nonexistent/file.bin")
        self.assertEqual(len(points), 0)

    def test_parser_stores_points(self):
        """Test that parser properly stores points in self.points."""
        test_packet = create_imu_packet(
            time_counter=0,
            time=1500.0,
            roll=15.0,
            pitch=25.0
        )
        
        test_file = Path(__file__).parent / "test_imu_store.bin"
        
        try:
            with open(test_file, 'wb') as f:
                f.write(test_packet)
            
            self.assertEqual(len(self.parser.points), 0)
            points = self.parser.parse_binary_file(str(test_file))
            self.assertEqual(len(self.parser.points), 1)
            self.assertEqual(self.parser.points, points)
        finally:
            if test_file.exists():
                test_file.unlink()

    def test_parse_real_test_data(self):
        """Test parsing actual IMU test data if it exists."""
        test_data_path = Path(__file__).parent / "data" / "example" / "IMU_Test_Data.bin"
        
        if test_data_path.exists():
            points = self.parser.parse_binary_file(str(test_data_path))
            # Should parse some points from real data
            self.assertGreater(len(points), 0)
            
            # Verify all points have required fields
            for point in points:
                self.assertIsInstance(point, IMUPoint)
                self.assertIsNotNone(point.time)
                self.assertIsNotNone(point.roll)
                self.assertIsNotNone(point.pitch)
                self.assertIsNotNone(point.heading)


if __name__ == '__main__':
    unittest.main()
