"""
Radar Parser Module - Convenience imports for radar data parsing and bag file creation.

Classes:
    - RadarParser: Parser for converting radar data to ROS bag files
"""

from .radar_parser_impl import RadarParser

if __name__ == "__main__":
    parser = RadarParser()

    # Parse hex-encoded test data
    hex_file = "tests/data/example/Radar_Test_Data.txt"

    points = parser.parse_hex_text(hex_file)

    # Create bag file
    try:
        import rosbag2_py
        bag_file = "radar_output"
        if parser.to_bag(bag_file, points):
            parser.inspect_bag(bag_file)
    except ImportError:
        print("\n Install ROS2 to create bag files:")
