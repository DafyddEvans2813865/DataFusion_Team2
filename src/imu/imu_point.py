from dataclasses import dataclass, field
from typing import Optional


@dataclass
class IMUPoint:
    """Single IMU data point."""
    time_counter: int
    time: float
    # Changed to Quaternion to avoid overhead 
    qx: float  
    qy: float  
    qz: float  
    qw: float  
    x_accel: float
    y_accel: float
    z_accel: float
    x_rate: float
    y_rate: float
    z_rate: float

    def to_dict(self) -> dict:
        return {
            'time_counter': self.time_counter,
            'time': self.time,
            'qx': self.qx,
            'qy': self.qy,
            'qz': self.qz,
            'qw': self.qw,
            'x_accel': self.x_accel,
            'y_accel': self.y_accel,
            'z_accel': self.z_accel,
            'x_rate': self.x_rate,
            'y_rate': self.y_rate,
            'z_rate': self.z_rate
        }
