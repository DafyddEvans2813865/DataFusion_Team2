import struct
import csv
import serial
import threading

# =========================
# SERIAL CONFIG
# =========================
RADAR_PORT = '/dev/tty.usbserial-RADAR'
IMU_PORT   = '/dev/tty.usbserial-IMU'

RADAR_BAUD = 921600
IMU_BAUD   = 115200

 # Magic word in raw bytes
magic = bytes.fromhex('0201040306050807')

frames = []
imu_data_list = []

# =========================
# HELPER FUNCTIONS
# =========================
def u32(data, i):
    return int.from_bytes(data[i:i+4], 'little')


def f32(data, i):
    return struct.unpack('<f', data[i:i+4])[0]


# =========================
# RADAR THREAD
# =========================
def radar_thread():
    ser = serial.Serial(RADAR_PORT, RADAR_BAUD, timeout=1)
    buffer = bytearray()

    print("Radar thread started...")

    while True:
        buffer += ser.read(4096)

        while True:
            start = buffer.find(magic)
            if start == -1 or len(buffer) < start + 16:
                break

            pkt_len = int.from_bytes(buffer[start+12:start+16], 'little')
            if len(buffer) < start + pkt_len:
                break

            p = buffer[start:start+pkt_len]

            header = {}
            header['version']          = u32(p, 8)
            header['packetLength']     = u32(p, 12)
            header['platform']         = u32(p, 16)
            header['frameNumber']      = u32(p, 20)
            header['time']             = u32(p, 24)
            header['numObjects']       = u32(p, 28)
            header['numTLV']           = u32(p, 32)
            header['subframeNum']      = u32(p, 36)
            header['numStaticObjects'] = u32(p, 40)

            frame_data = {
                "header": header,
                "dynamic_points": [],
                "dynamic_sideinfo": [],
                "static_points": [],
                "static_sideinfo": [],
                "tracks": [],
                "associations": []
            }

            idx = 44

            for tlv in range(header['numTLV']):
                if idx + 8 > len(p):
                    break

                tlv_type   = u32(p, idx)
                tlv_length = u32(p, idx + 4)

                tlv_start = idx
                tlv_end   = tlv_start + 8 + tlv_length

                if tlv_length < 8 or tlv_end > len(p):
                    break

                idx += 8

                if tlv_type == 1:
                    count = tlv_length // 16
                    for k in range(count):
                        r  = f32(p, idx)
                        a  = f32(p, idx + 4)
                        e  = f32(p, idx + 8)
                        d  = f32(p, idx + 12)

                        frame_data["dynamic_points"].append({
                            "range": r,
                            "angle": a,
                            "elev": e,
                            "doppler": d
                        })
                        idx += 16
                    idx = tlv_end

                else:
                    idx = tlv_end

            frames.append(frame_data)
            print(f"[RADAR] Frame {header['frameNumber']} Points: {len(frame_data['dynamic_points'])}")

            buffer = buffer[start + pkt_len:]

# =========================
# IMU THREAD
# =========================
def imu_thread():
    ser = serial.Serial(IMU_PORT, IMU_BAUD, timeout=1)
    buffer = bytearray()

    IMU_FMT = '<IdfffffffffffffffBBB'
    IMU_SIZE = struct.calcsize(IMU_FMT)
    IMU_FIELDS = [
        'timeCntr', 'time',
        'roll', 'pitch', 'heading',
        'xAccel', 'yAccel', 'zAccel',
        'xRate', 'yRate', 'zRate',
        'xRateBias', 'yRateBias', 'zRateBias',
        'xMag', 'yMag', 'zMag',
        'opMode', 'linAccSw', 'turnSw'
    ]

    print("IMU thread started...")

    while True:
        buffer += ser.read(512)

        while True:
            if len(buffer) < 6:
                break

            # Find header anywhere in buffer
            start = buffer.find(b'\x55\x55')
            if start == -1:
                buffer.clear()
                break

            # Align buffer to header
            if start > 0:
                buffer = buffer[start:]

            if len(buffer) < 6:
                break

            length = buffer[4]

            # Ensure full packet is available
            if len(buffer) < 5 + length + 2:
                break

            # Only accept packets large enough for struct
            if length < IMU_SIZE:
                buffer = buffer[1:]
                continue

            # Extract only struct portion (ignore checksum/extra)
            payload = buffer[5:5+IMU_SIZE]
            total_len = 5 + length + 2

            try:
                values = struct.unpack(IMU_FMT, payload)
                data = dict(zip(IMU_FIELDS, values))
                imu_data_list.append(data)

                print(f"[IMU] t={data['time']:.2f} Roll={data['roll']:.2f} Pitch={data['pitch']:.2f}")

            except Exception as e:
                print(f"IMU parse error (len={len(payload)})")

            buffer = buffer[total_len:]

# =========================
# START THREADS
# =========================
threading.Thread(target=radar_thread, daemon=True).start()
threading.Thread(target=imu_thread, daemon=True).start()

# Keep main alive
while True:
    pass
