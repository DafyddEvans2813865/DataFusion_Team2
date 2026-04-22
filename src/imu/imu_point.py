from dataclasses import dataclass, field
from typing import Optional


@dataclass
class IMUPoint:
    """Single IMU data point."""
    time_counter: int
    time: float
    roll: float
    pitch: float
    yaw: float  # heading/yaw in degrees
    x_accel: float
    y_accel: float
    z_accel: float
    x_rate: float
    y_rate: float
    z_rate: float
    x_rate_bias: float = 0.0
    y_rate_bias: float = 0.0
    z_rate_bias: float = 0.0
    x_mag: float = 0.0
    y_mag: float = 0.0
    z_mag: float = 0.0
    op_mode: int = 0
    lin_acc_switch: int = 0
    turn_switch: int = 0

    def to_dict(self) -> dict:
        return {
            'time_counter': self.time_counter,
            'time': self.time,
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'x_accel': self.x_accel,
            'y_accel': self.y_accel,
            'z_accel': self.z_accel,
            'x_rate': self.x_rate,
            'y_rate': self.y_rate,
            'z_rate': self.z_rate,
            'x_rate_bias': self.x_rate_bias,
            'y_rate_bias': self.y_rate_bias,
            'z_rate_bias': self.z_rate_bias,
            'x_mag': self.x_mag,
            'y_mag': self.y_mag,
            'z_mag': self.z_mag,
            'op_mode': self.op_mode,
            'lin_acc_switch': self.lin_acc_switch,
            'turn_switch': self.turn_switch
        }
