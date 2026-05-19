#!/usr/bin/env python3
import sys
import shutil
import rclpy
from pathlib import Path
from typing import Optional

from radar.radar_parser_impl import RadarParser
from imu.imu_parser_impl import IMUParser

def remove_existing_bag(output_file) -> None:
    output_path = Path(output_file)
    if output_path.exists():
        if output_path.is_dir():
            shutil.rmtree(output_path)
        else:
            output_path.unlink()
        print("Removed Previous existing .bag file")

def convert_radar_to_bag(input_file: str, topic_name: str) -> bool:
    print("\n" + "=" * 60)
    print("RADAR DATA CONVERSION")
    print("=" * 60)
    parser = RadarParser(topic_name)
    
    if not Path(input_file).exists():
        print(f"Input file not found: {input_file}")
        return False
    
    print(f"Parsing radar data from: {input_file}")
    points = parser.parse_hex_text(input_file)
    
    if not points:
        print("No radar points parsed")
        return False
    
    print(f"Parsed {len(points)} radar detection points")
    
    # Count by frame
    frames = {}
    for point in points:
        if point.frame not in frames:
            frames[point.frame] = 0
        frames[point.frame] += 1
    
    print(f"Frames: {len(frames)} (range: {min(frames.keys())}-{max(frames.keys())})")
    print(f"Samples per frame: avg={len(points)/len(frames):.1f}, "
          f"min={min(frames.values())}, max={max(frames.values())}")
    """
    # Create bag file
    try:
        import rosbag2_py
        print(f"\nCreating ROS2 bag file: {output_file}")
        remove_existing_bag(output_file)
        if parser.to_bag(output_file):
            parser.inspect_bag(output_file)
            return True
        else:
            print("Failed to create radar bag file")
            return False
    except ImportError:
        print("\nROS2 not installed")
        print("\nTo use bag file export, install ROS2 (rosbag2_py):")
        return False
        """





def convert_imu_to_bag(IMU_PORT: str, IMU_BAUD: int, output_file: str = "imu_output.bag") -> bool:
    print("\n" + "=" * 60)
    print("IMU DATA STREAMING TO BAG (A2 MODE)")
    print("=" * 60)

    try:
        import serial
        
        print(f"Opening serial port: {IMU_PORT} @ {IMU_BAUD} baud")
        remove_existing_bag(output_file)
        serial_port = serial.Serial(IMU_PORT, IMU_BAUD, timeout=5) #5 sec time out 
        
        parser = IMUParser()
        
        print(f"Recording to: {output_file}")
        
        # Stream directly to bag file
        success = parser.to_bag_multithreaded(output_file, topic_name="/imu/data", serial_port=serial_port)
        
        serial_port.close()
        return success
        
    except ImportError as e:
        print(f"\nMissing package: {e}")
        if "serial" in str(e).lower():
            print("Install pyserial: pip install pyserial")
        else:
            print("Install ROS2: https://docs.ros.org/en/humble/Installation/")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False


def main():
    print("\n" + "=" * 60)
    print("SENSOR DATA TO ROS BAG CONVERSION")
    print("=" * 60)
    
    # Define input/output files Data 
    radar_input = "tests/data/example/Radar_Test_Data.txt"
    radar_topic_1="/radar/points1"
    radar_topic_2="radar/points2"
    
    # IMU serial config
    IMU_PORT = '/dev/ttyUSB0'
    IMU_BAUD = 115200
    
    results = {'radar1': False,'radar2': False,'imu': False}
    
    rclpy.init()
    # Convert radar data
    results['radar1'] = convert_radar_to_bag(radar_input, radar_topic_1)
    results['radar2'] = convert_radar_to_bag(radar_input, radar_topic_2)

    # Capture IMU data over serial for CAPTURE_DURATION seconds, then write bag
    results['imu'] = convert_imu_to_bag(IMU_PORT,IMU_BAUD)
    rclpy.shutdown()

    # Summary
    print("\n" + "=" * 60)
    print("CONVERSION SUMMARY")
    print("=" * 60)
    
    for sensor, success in results.items():
        status = "SUCCESS" if success else "FAILED"
        print(f"{sensor.upper():15} {status}")
    
    print("\n" + "=" * 60)
    if all(results.values()):
        print("All conversions successful!")
        print("\nTo play the ROS2 bag files:")
        print(f"  ros2 bag play {radar_output}")
        print(f"  ros2 bag play {imu_output}")
        print("\nOr visualize in RViz2:")
        print("  rviz2")
        print(f"\nVerify IMU parsing with CSV export:")
        print(f"  Check {imu_csv_export}")
        return 0
    elif any(results.values()):
        print("Some conversions completed, but not all succeeded")
        return 1
    else:
        print("\nNo conversions completed successfully")
        return 1


if __name__ == "__main__":
    sys.exit(main())
