"""
IMU Parser Module - Convenience imports for IMU data parsing.

Classes:
    - IMUParser: Parser for IMU binary data files
"""

from .imu_parser_impl import IMUParser

if __name__ == "__main__":
    parser = IMUParser()
    
    # Parse binary IMU data
    imu_file = "tests/data/example/IMU_Test_Data.bin"
    
    points = parser.parse_binary_file(imu_file)
    
    print(f"Parsed {len(points)} IMU data points")

    # Create bag file
    try:
        import rosbag
        bag_file = "imu_output.bag"
        if parser.to_bag(bag_file):
            parser.inspect_bag(bag_file)
    except ImportError:
        print("\nInstall ROS to create bag files:")
        print("  pip install rosbag rospy geometry_msgs sensor_msgs")
