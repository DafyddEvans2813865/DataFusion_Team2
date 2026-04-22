import sys
import unittest
from pathlib import Path

# Add src to path for imu imports 
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from imu.imu_point import IMUPoint
from imu.imu_parser_impl import IMUParser


class TestIMUPoint(unittest.TestCase):
    def test_imu_point_creation(self):
        point = IMUPoint(
            time_counter=0,
            time=1000.0,
            roll=10.5,
            pitch=20.3,
            yaw=45.0,
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
            yaw=20.0,
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

    def test_parse_a2_csv_file_not_found(self):
        """Test parsing A2 CSV when file doesn't exist."""
        points = self.parser.parse_a2_csv_file("/nonexistent/file.csv")
        self.assertEqual(len(points), 0)

    def test_parse_a2_csv_real_data(self):
        """Test parsing actual A2 CSV test data if it exists."""
        test_data_path = Path(__file__).parent / "data" / "example" / "a2_packet_type_a2.csv"
        
        if test_data_path.exists():
            points = self.parser.parse_a2_csv_file(str(test_data_path))
            # Should parse some points from real data
            self.assertGreater(len(points), 0)
            
            # Verify all points have required fields
            for point in points:
                self.assertIsInstance(point, IMUPoint)
                self.assertIsNotNone(point.time)
                self.assertIsNotNone(point.roll)
                self.assertIsNotNone(point.pitch)
                self.assertIsNotNone(point.yaw)


    def test_parse_a2_binary_file_not_found(self):
        """Test parsing A2 binary when file doesn't exist."""
        points = self.parser.parse_a2_binary_file("/nonexistent/file.bin")
        self.assertEqual(len(points), 0)

    def test_parse_a2_binary_real_data(self):
        """Test parsing actual A2 binary test data if it exists."""
        test_data_path = Path(__file__).parent / "data" / "example" / "stationary_A2.bin"
        
        if test_data_path.exists():
            points = self.parser.parse_a2_binary_file(str(test_data_path))
            # Should parse multiple packets from real binary data
            self.assertGreater(len(points), 0)
            
            # Verify all points have required fields
            for point in points:
                self.assertIsInstance(point, IMUPoint)
                self.assertIsNotNone(point.time)
                self.assertIsNotNone(point.roll)
                self.assertIsNotNone(point.pitch)
                self.assertIsNotNone(point.yaw)
            
            # Check expected count (2001 packets in the test file)
            self.assertEqual(len(points), 2001)


if __name__ == '__main__':
    unittest.main()
