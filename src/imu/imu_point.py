from dataclasses import dataclass


@dataclass
class IMUPoint:
    """Single IMU data point."""
    time_counter: int
    time: float
    roll: float
    pitch: float
    heading: float
    x_accel: float
    y_accel: float
    z_accel: float
    x_rate: float
    y_rate: float
    z_rate: float
    x_rate_bias: float
    y_rate_bias: float
    z_rate_bias: float
    x_mag: float
    y_mag: float
    z_mag: float
    op_mode: int
    lin_acc_switch: int
    turn_switch: int

    def to_dict(self) -> dict:
        return {
            'time_counter': self.time_counter,
            'time': self.time,
            'roll': self.roll,
            'pitch': self.pitch,
            'heading': self.heading,
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
            'turn_switch': self.turn_switch,
        }
