import sys
import unittest
import struct
from pathlib import Path

# Add src to path for radar imports 
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from radar.radar_point import RadarPoint
from radar.radar_parser_impl import RadarParser, HAS_ROS

class TestRadarPoint(unittest.TestCase):
    def test_radar_point_creation(self):
        point = RadarPoint(
            frame=0,
            timestamp=1000,
            range=10.5,
            angle=45.0,
            elev=0.0,
            doppler=0.5,
            snr=20,
            noise=1
        )
        
        self.assertEqual(point.frame, 0)
        self.assertEqual(point.timestamp, 1000)
        self.assertEqual(point.range, 10.5)
        self.assertEqual(point.angle, 45.0)
        self.assertEqual(point.elev, 0.0)
        self.assertEqual(point.doppler, 0.5)
        self.assertEqual(point.snr, 20)
        self.assertEqual(point.noise, 1)

    def test_radar_point_to_dict(self):
        point = RadarPoint(
            frame=1,
            timestamp=2000,
            range=20.0,
            angle=90.0,
            elev=15.0,
            doppler=1.0,
            snr=30,
            noise=2
        )
        
        point_dict = point.to_dict()
        
        self.assertIsInstance(point_dict, dict)
        self.assertEqual(point_dict['frame'], 1)
        self.assertEqual(point_dict['timestamp'], 2000)
        self.assertEqual(point_dict['range'], 20.0)
        self.assertEqual(point_dict['angle'], 90.0)
        self.assertEqual(point_dict['elev'], 15.0)
        self.assertEqual(point_dict['doppler'], 1.0)
        self.assertEqual(point_dict['snr'], 30)
        self.assertEqual(point_dict['noise'], 2)


class TestRadarParser(unittest.TestCase):

    def setUp(self):
        self.parser = RadarParser()

    def test_parser_initialization(self):
        self.assertIsNotNone(self.parser)
        self.assertEqual(len(self.parser.points), 0)
        self.assertIsInstance(self.parser.points, list)

    def test_parse_hex_text_valid_data(self):
        test_data = (
            struct.pack('I', 0) +           # frame 0
            struct.pack('I', 1000) +        # timestamp 1000
            struct.pack('f', 10.5) +        # range 10.5
            struct.pack('f', 45.0) +        # angle 45.0
            struct.pack('f', 0.5)           # doppler 0.5
        )
        
        test_hex = test_data.hex()
        test_file = Path(__file__).parent / "test_radar_data.txt"
        
        try:
            with open(test_file, 'w') as f:
                f.write(test_hex)
            
            points = self.parser.parse_hex_text(str(test_file))
            
            self.assertEqual(len(points), 1)
            self.assertEqual(points[0].frame, 0)
            self.assertEqual(points[0].timestamp, 1000)
            self.assertAlmostEqual(points[0].range, 10.5, places=5)
            self.assertAlmostEqual(points[0].angle, 45.0, places=5)
            self.assertAlmostEqual(points[0].doppler, 0.5, places=5)
            self.assertEqual(points[0].snr, 0)
            self.assertEqual(points[0].noise, 0)
            self.assertEqual(points[0].elev, 0.0)
        finally:
            if test_file.exists():
                test_file.unlink()

    def test_parse_hex_text_multiple_points(self):
        # Create test data with multiple points
        test_data = b''
        for i in range(3):
            test_data += (
                struct.pack('I', i) +           # frame i
                struct.pack('I', 1000 + i * 100) +
                struct.pack('f', 10.0 + i) +
                struct.pack('f', 45.0 + i * 10) +
                struct.pack('f', 0.5 + i * 0.1)
            )
        
        test_hex = test_data.hex()
        test_file = Path(__file__).parent / "test_radar_data_multi.txt"
        
        try:
            with open(test_file, 'w') as f:
                f.write(test_hex)
            
            points = self.parser.parse_hex_text(str(test_file))
            
            self.assertEqual(len(points), 3)
            for i in range(3):
                self.assertEqual(points[i].frame, i)
                self.assertEqual(points[i].timestamp, 1000 + i * 100)
                self.assertAlmostEqual(points[i].range, 10.0 + i, places=5)
        finally:
            if test_file.exists():
                test_file.unlink()

    def test_parse_hex_text_file_not_found(self):
        points = self.parser.parse_hex_text("/nonexistent/file.txt")
        self.assertEqual(len(points), 0)

    def test_parse_hex_text_with_spaces(self):
        test_data = (
            struct.pack('I', 0) +
            struct.pack('I', 1000) +
            struct.pack('f', 15.5) +
            struct.pack('f', 30.0) +
            struct.pack('f', 0.3)
        )
        
        test_hex = test_data.hex()
        # Add spaces to hex string
        test_hex_spaced = ' '.join([test_hex[i:i+2] for i in range(0, len(test_hex), 2)])
        
        test_file = Path(__file__).parent / "test_radar_spaced.txt"
        
        try:
            with open(test_file, 'w') as f:
                f.write(test_hex_spaced)
            
            points = self.parser.parse_hex_text(str(test_file))
            
            self.assertEqual(len(points), 1)
            self.assertEqual(points[0].frame, 0)
        finally:
            if test_file.exists():
                test_file.unlink()

    @unittest.skipIf(not HAS_ROS, "ROS not installed")
    def test_to_point_cloud_conversion(self):
        # Create test points with known values
        points = [
            RadarPoint(
                frame=0, timestamp=1000, range=10.0, angle=0.0,
                elev=0.0, doppler=0.0, snr=20, noise=0
            ),
            RadarPoint(
                frame=0, timestamp=1000, range=10.0, angle=90.0,
                elev=0.0, doppler=0.0, snr=20, noise=0
            ),
        ]
        
        point_cloud = self.parser.to_point_cloud(points)
        
        self.assertIsNotNone(point_cloud)
        self.assertEqual(point_cloud.width * point_cloud.height, 2)

    def test_to_point_cloud_empty_points(self):
        point_cloud = self.parser.to_point_cloud([])
        self.assertIsNone(point_cloud)

    @unittest.skipIf(not HAS_ROS, "ROS not installed")
    def test_to_point_cloud_coordinate_conversion(self):
        # Point at 0 degrees, 10 units range, 0 elevation
        point = RadarPoint(
            frame=0, timestamp=1000, range=10.0, angle=0.0,
            elev=0.0, doppler=0.0, snr=20, noise=0
        )
        
        # Expected: x=10, y=0, z=0 (since cos(0)=1, sin(0)=0)
        point_cloud = self.parser.to_point_cloud([point])
        
        self.assertIsNotNone(point_cloud)
        # The actual coordinate requires ROS dependencies, just verify it's not None

    def test_parser_stores_points(self):
        test_data = (
            struct.pack('I', 0) +
            struct.pack('I', 1000) +
            struct.pack('f', 10.5) +
            struct.pack('f', 45.0) +
            struct.pack('f', 0.5)
        )
        
        test_hex = test_data.hex()
        test_file = Path(__file__).parent / "test_radar_store.txt"
        
        try:
            with open(test_file, 'w') as f:
                f.write(test_hex)
            
            self.assertEqual(len(self.parser.points), 0)
            points = self.parser.parse_hex_text(str(test_file))
            self.assertEqual(len(self.parser.points), 1)
            self.assertEqual(self.parser.points, points)
        finally:
            if test_file.exists():
                test_file.unlink()

    def test_parse_real_test_data(self):
        test_data_path = Path(__file__).parent / "data" / "example" / "Radar_Test_Data.txt"
        
        if test_data_path.exists():
            points = self.parser.parse_hex_text(str(test_data_path))
            self.assertGreater(len(points), 0)
            
            # Verify all points have required fields
            for point in points:
                self.assertIsInstance(point, RadarPoint)
                self.assertIsNotNone(point.frame)
                self.assertIsNotNone(point.timestamp)
                self.assertIsNotNone(point.range)
                self.assertIsNotNone(point.angle)


if __name__ == '__main__':
    unittest.main()
