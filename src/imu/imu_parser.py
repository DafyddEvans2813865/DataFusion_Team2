"""
IMU Parser Module - Convenience imports for IMU data parsing.

Classes:
    - IMUParser: Parser for IMU binary data files
"""

from .imu_parser_impl import IMUParser

if __name__ == "__main__":
    parser = IMUParser()
    
    # Parse binary IMU data
    imu_file = "tests/data/example/20_second_moving_test.bin"
    
    points = parser.parse_binary_file(imu_file)
    
    print(f"Parsed {len(points)} IMU data points")
    
    if len(points) > 0:
        print(f"First point: time={points[0].time:.2f}, roll={points[0].roll:.2f}")
        print(f"Last point: time={points[-1].time:.2f}, roll={points[-1].roll:.2f}")
