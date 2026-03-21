from dataclasses import dataclass


@dataclass
class RadarDetection:
    """Single radar detection point."""
    frame: int
    timestamp: int
    range: float
    angle: float
    elev: float
    doppler: float
    snr: int
    noise: int

    def to_dict(self) -> dict:
        """Convert detection to dictionary."""
        return {
            'frame': self.frame,
            'timestamp': self.timestamp,
            'range': self.range,
            'angle': self.angle,
            'elev': self.elev,
            'doppler': self.doppler,
            'snr': self.snr,
            'noise': self.noise
        }
