import struct
from pathlib import Path
from typing import List, Optional

from .imu_point import IMUPoint


class IMUParser:
    """Parser for IMU binary data with 0x5555 packet headers."""
    
    # IMU packet structure: [0x55 0x55][type(2)][length(1)][payload][checksum(2)]
    IMU_FMT = '<IdfffffffffffffffBBB'
    IMU_HEADER = b'\x55\x55'
    
    def __init__(self):
        self.points: List[IMUPoint] = []
        self.imu_size = struct.calcsize(self.IMU_FMT)
        self.imu_fields = [
            'time_counter', 'time',
            'roll', 'pitch', 'heading',
            'x_accel', 'y_accel', 'z_accel',
            'x_rate', 'y_rate', 'z_rate',
            'x_rate_bias', 'y_rate_bias', 'z_rate_bias',
            'x_mag', 'y_mag', 'z_mag',
            'op_mode', 'lin_acc_switch', 'turn_switch'
        ]
    
    def parse_binary_file(self, binary_file_path: str) -> List[IMUPoint]:
        """Parse binary IMU data file and extract IMU data points."""
        points = []
        
        try:
            with open(binary_file_path, 'rb') as f:
                raw_bytes = f.read()
            
            buffer = bytearray(raw_bytes)
            
            while buffer:
                # Find header
                start = buffer.find(self.IMU_HEADER)
                if start == -1:
                    break
                
                # Need at least 5 bytes: header(2) + type(2) + length(1)
                if len(buffer) < start + 5:
                    break
                
                # Align buffer to header
                if start > 0:
                    buffer = buffer[start:]
                
                if len(buffer) < 5:
                    break
                
                # Extract length at offset 4
                length = buffer[4]
                
                # Full packet: header(2) + type(2) + length(1) + payload(length) + checksum(2)
                total_len = 5 + length + 2
                
                # Ensure we have full packet
                if len(buffer) < total_len:
                    break
                
                # Extract payload
                payload = buffer[5:5+self.imu_size]
                
                if len(payload) < self.imu_size:
                    buffer = buffer[1:]
                    continue
                
                try:
                    values = struct.unpack(self.IMU_FMT, payload[:self.imu_size])
                    
                    # Create IMUPoint from parsed values
                    point = IMUPoint(
                        time_counter=values[0],
                        time=values[1],
                        roll=values[2],
                        pitch=values[3],
                        heading=values[4],
                        x_accel=values[5],
                        y_accel=values[6],
                        z_accel=values[7],
                        x_rate=values[8],
                        y_rate=values[9],
                        z_rate=values[10],
                        x_rate_bias=values[11],
                        y_rate_bias=values[12],
                        z_rate_bias=values[13],
                        x_mag=values[14],
                        y_mag=values[15],
                        z_mag=values[16],
                        op_mode=values[17],
                        lin_acc_switch=values[18],
                        turn_switch=values[19]
                    )
                    points.append(point)
                
                except struct.error as e:
                    print(f"IMU parse error: {e}")
                
                # Move to next packet
                buffer = buffer[total_len:]
        
        except FileNotFoundError:
            print(f"File not found: {binary_file_path}")
            return []
        except Exception as e:
            print(f"Error parsing IMU file: {e}")
            return []
        
        self.points = points
        return points
