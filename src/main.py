#!/usr/bin/env python3
import sys
import shutil
import struct
import serial
from pathlib import Path
from typing import Optional

from radar.radar_parser_impl import RadarParser
from imu.imu_parser_impl import IMUParser

MAGIC_WORD = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
HEADER_SIZE = 40
CLI_BAUD = 115200
DATA_BAUD = 921600

def remove_existing_bag(output_file) -> None:
    output_path = Path(output_file)
    if output_path.exists():
        if output_path.is_dir():
            shutil.rmtree(output_path)
        else:
            output_path.unlink()
        print("Removed Previous existing .bag file")


def send_radar_config(cfg_file: str, cli_port: str) -> bool:
    """Send TI mmWave cfg file over CLI UART."""

    try:
        cli = serial.Serial(cli_port, CLI_BAUD, timeout=1)
    except Exception as e:
        print(f"Failed to open CLI port: {e}")
        return False

    try:
        with open(cfg_file, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"Radar config not found: {cfg_file}")
        cli.close()
        return False

    print(f"Sending radar config: {cfg_file}")

    for line in lines:
        line = line.strip()

        if not line or line.startswith('%'):
            continue

        print(f"> {line}")
        cli.write((line + '\n').encode())
        cli.read(cli.in_waiting or 1)

    cli.close()
    print("Radar config complete")
    return True



def read_radar_frame(ser, buffer: bytes):
    """Read one complete TI radar UART frame."""

    MAX_BUF = 2**18

    while True:

        n = ser.in_waiting

        if n > 0:
            buffer += ser.read(n)
        elif len(buffer) < HEADER_SIZE + 8:
            chunk = ser.read(64)

            if not chunk:
                return None, buffer

            buffer += chunk

        idx = buffer.find(MAGIC_WORD)

        if idx == -1:
            buffer = buffer[-7:]
            continue

        if idx > 0:
            buffer = buffer[idx:]

        if len(buffer) < HEADER_SIZE:
            continue

        total_len = struct.unpack_from('<I', buffer, 12)[0]

        if total_len == 0 or total_len > MAX_BUF:
            buffer = buffer[8:]
            continue

        while len(buffer) < total_len:
            chunk = ser.read(min(total_len - len(buffer), 8192))

            if not chunk:
                return None, buffer

            buffer += chunk

        raw = buffer[:total_len]
        buffer = buffer[total_len:]

        return raw, buffer

def convert_radar_to_bag(
    cli_port: str,
    data_port: str,
    cfg_file: str,
    output_file: str = "radar_output.bag"
) -> bool:

    print("\n" + "=" * 60)
    print("REALTIME RADAR STREAM TO ROS2 BAG")
    print("=" * 60)

    if not Path(cfg_file).exists():
        print(f"Radar config file not found: {cfg_file}")
        return False

    if not send_radar_config(cfg_file, cli_port):
        return False

    try:
        print(f"Opening radar data port: {data_port}")

        data_serial = serial.Serial(data_port, DATA_BAUD, timeout=5)
        data_serial.set_buffer_size(rx_size=2**17)

        parser = RadarParser()

        remove_existing_bag(output_file)

        print("Streaming radar data...")
        print("Press Ctrl+C to stop capture")

        buffer = b''
        frame_count = 0

        while True:

            frame, buffer = read_radar_frame(data_serial, buffer)

            if frame is None:
                continue

            try:
                parser._parse_packet(frame)
                frame_count += 1

                if frame_count % 10 == 0:
                    print(f"Frames captured: {frame_count}")

            except Exception as e:
                print(f"Frame parse error: {e}")

    except KeyboardInterrupt:
        print("\nStopping radar capture...")

    except Exception as e:
        print(f"Radar error: {e}")
        return False

    finally:
        try:
            data_serial.close()
        except:
            pass

    try:
        print(f"Writing ROS2 bag: {output_file}")

        if parser.to_bag(output_file):
            parser.inspect_bag(output_file)
            return True

        return False

    except Exception as e:
        print(f"Bag export failed: {e}")
        return False





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
    
    # Radar UART configuration
    CLI_PORT = '/dev/ttyUSB0'
    DATA_PORT = '/dev/ttyUSB1'
    RADAR_CFG = 'radar_config.cfg'
    radar_output = 'radar_output.bag'
    
    # IMU serial config
    IMU_PORT = '/dev/ttyUSB0'
    IMU_BAUD = 115200
    
    results = {'radar': False,'imu': False}
    
    # Realtime radar capture
    results['radar'] = convert_radar_to_bag(
        CLI_PORT,
        DATA_PORT,
        RADAR_CFG,
        radar_output
    )

    # Capture IMU data over serial for CAPTURE_DURATION seconds, then write bag
    results['imu'] = convert_imu_to_bag(IMU_PORT,IMU_BAUD)
    
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
        print("\nOr visualize in RViz2:")
        print("  rviz2")
        return 0
    elif any(results.values()):
        print("Some conversions completed, but not all succeeded")
        return 1
    else:
        print("\nNo conversions completed successfully")
        return 1


if __name__ == "__main__":
    sys.exit(main())
