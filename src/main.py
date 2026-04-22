#!/usr/bin/env python3
import sys
from pathlib import Path
from typing import Optional

from radar.radar_parser_impl import RadarParser
from imu.imu_parser_impl import IMUParser


def convert_radar_to_bag(input_file: str, output_file: str = "radar_output.bag") -> bool:
    print("\n" + "=" * 60)
    print("RADAR DATA CONVERSION")
    print("=" * 60)
    parser = RadarParser()
    
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
    
    # Create bag file
    try:
        import rosbag
        print(f"\nCreating bag file: {output_file}")
        if parser.to_bag(output_file):
            parser.inspect_bag(output_file)
            return True
        else:
            print("Failed to create radar bag file")
            return False
    except ImportError:
        print("\nROS not installed")
        print("\nTo use bag file export, install ROS:")
        return False


def convert_imu_to_bag(input_file: str, output_file: str = "imu_output.bag", csv_export: Optional[str] = None) -> bool:
    print("\n" + "=" * 60)
    print("IMU DATA CONVERSION (A2 MODE)")
    print("=" * 60)
    
    parser = IMUParser()
    
    if not Path(input_file).exists():
        print(f"Input file not found: {input_file}")
        return False
    
    print(f"Parsing IMU A2 Binary from: {input_file}")
    points = parser.parse_a2_binary_file(input_file)
    
    if not points:
        print("No IMU points parsed")
        return False
    
    print(f"Parsed {len(points)} IMU data points")
    
    # Show time range and statistics
    if points:
        time_min = min(p.time for p in points)
        time_max = max(p.time for p in points)
        duration = time_max - time_min
        print(f"Time range: {time_min:.2f}s to {time_max:.2f}s (duration: {duration:.2f}s)")
        print(f"Sample rate: {len(points)/duration:.1f} Hz" if duration > 0 else "")
    
    # Export to CSV for verification if requested
    if csv_export:
        print(f"\nExporting parsed data to CSV for verification: {csv_export}")
        if parser.to_csv(csv_export):
            print(f"CSV verification export successful")
        else:
            print(f"CSV verification export failed")
    
    # Create bag file
    try:
        import rosbag
        print(f"\nCreating ROS bag file: {output_file}")
        if parser.to_bag(output_file):
            parser.inspect_bag(output_file)
            return True
        else:
            print("Failed to create IMU bag file")
            return False
    except ImportError:
        print("\nROS not installed")
        print("\nTo use bag file export, install ROS:")
        print("  macOS: https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html")
        print("  Ubuntu: sudo apt install python3-rosbag2 python3-rclpy python3-geometry-msgs")
        return False


def main():
    print("\n" + "=" * 60)
    print("SENSOR DATA TO ROS BAG CONVERSION")
    print("=" * 60)
    
    # Define input/output files Data 
    radar_input = "tests/data/example/Radar_Test_Data.txt"
    radar_output = "radar_output.bag"
    
    # Define input/output files IMU 
    imu_input = "tests/data/example/IMU_Test_Data.bin"
    imu_output = "imu_output.bag"
    imu_csv_export = "tests/data/example/IMU_Test_Output.csv"  # CSV export for verification
    
    results = {
        'radar': False,
        'imu': False
    }
    
    # Convert radar data
    if Path(radar_input).exists():
        results['radar'] = convert_radar_to_bag(radar_input, radar_output)
    else:
        print(f"\nRadar test file not found: {radar_input}")
    
    # Convert IMU binary data to bag file with verification export
    if Path(imu_input).exists():
        results['imu'] = convert_imu_to_bag(
            imu_input, 
            imu_output,
            csv_export=imu_csv_export
        )
    else:
        print(f"\nIMU test file not found: {imu_input}")
    
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
        print("\nTo play the ROS bag files:")
        print(f"  rosbag play {radar_output}")
        print(f"  rosbag play {imu_output}")
        print("\nOr visualize in RViz:")
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
