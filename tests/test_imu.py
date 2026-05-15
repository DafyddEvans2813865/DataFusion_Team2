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


if __name__ == '__main__':
    unittest.main()
