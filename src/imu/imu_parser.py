"""
IMU Parser Module - Convenience imports for IMU A2 binary data parsing.

Classes:
    - IMUParser: Parser for OpenIMU300ZI A2 mode binary data files
"""

from .imu_parser_impl import IMUParser

if __name__ == "__main__":
    parser = IMUParser()
    
    # Parse A2 mode binary IMU data
    imu_bin_file = "tests/data/example/stationary_A2.bin"
    
    points = parser.parse_a2_binary_file(imu_bin_file)
    
    print(f"Parsed {len(points)} IMU data points (A2 Binary Mode)")
    
    # Export parsed data to CSV for verification
    parser.to_csv("imu_parsed_output.csv")

    # Create bag file
    try:
        import rosbag
        bag_file = "imu_output.bag"
        if parser.to_bag(bag_file):
            print(f"Created ROS bag file: {bag_file}")
    except ImportError:
        print("\nInstall ROS to create bag files:")
        print("  pip install rosbag rospy geometry_msgs sensor_msgs")
