
import struct
import csv
import serial
import threading

# =========================
# HELPER FUNCTIONS
# =========================
def u32(data, i):
    return int.from_bytes(data[i:i+4], 'little')


def f32(data, i):
    return struct.unpack('<f', data[i:i+4])[0]

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
# RADAR THREAD (FILE SIMULATION)
# =========================
def radar_thread():
    print("Radar file simulation started...")

    with open('demoData.txt', 'r') as f:
        hex_data = f.read()

    raw_bytes = bytes.fromhex(hex_data)
    buffer = bytearray()
    idx_file = 0

    while True:
        # simulate streaming chunks
        chunk = raw_bytes[idx_file:idx_file+512]
        if not chunk:
            print("Radar file reached end. Stopping thread.")
            break

        buffer += chunk
        idx_file += 512

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
            buffer = buffer[start + pkt_len:]
        if idx_file >= len(raw_bytes):
            break
        import time
        time.sleep(0.02)

# =========================
# IMU THREAD (FILE SIMULATION)
# =========================
def imu_thread():
    print("IMU file simulation started...")

    with open('20_second_moving_test.bin', 'rb') as f:
        raw_bytes = f.read()
    print(f"IMU file size: {len(raw_bytes)} bytes")
    print("First 32 bytes:", raw_bytes[:32].hex())
    print("Header occurrences (0x5555):", raw_bytes.count(b'\x55\x55'))

    buffer = bytearray()
    idx_file = 0

    # Use original IMU structure (from working script)
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

    while True:
        chunk = raw_bytes[idx_file:idx_file+128]
        if not chunk:
            print("IMU file reached end. Stopping thread.")
            break

        buffer += chunk
        idx_file += 128

        while True:
            if len(buffer) < 6:
                break

            # Debug: inspect header
            # print(buffer[:10].hex())

            # Find header anywhere in buffer
            start = buffer.find(b'\x55\x55')
            if start != -1:
                print(f"IMU header found at buffer index {start}")
            if start == -1:
                buffer.clear()
                break

            # Align buffer to header
            if start > 0:
                buffer = buffer[start:]

            if len(buffer) < 6:
                break

            # Correct alignment: header is NOT just 2 bytes
            # Actual structure: [0x55 0x55][type(2)][length(1)][payload][checksum(2)]

            if len(buffer) < 7:
                break

            length = buffer[4]

            # Ensure we have full packet
            if len(buffer) < 5 + length + 2:
                break

            # Payload starts at byte 5
            payload = buffer[5:5+IMU_SIZE]
            total_len = 5 + length + 2

            if len(payload) < IMU_SIZE:
                buffer = buffer[1:]
                continue

            try:
                values = struct.unpack(IMU_FMT, payload[:IMU_SIZE])
                data = dict(zip(IMU_FIELDS, values))

                imu_data_list.append(data)

                print(f"[IMU] t={data['time']:.2f} Roll={data['roll']:.2f} Pitch={data['pitch']:.2f} AccelX={data['xAccel']:.3f}")

            except Exception as e:
                print(f"IMU parse error (len={len(payload)})")

            # Move forward one full fixed-size packet
            buffer = buffer[total_len:]
        if idx_file >= len(raw_bytes):
            break
        import time
        time.sleep(0.01)

# =========================
# START THREADS
# =========================
threading.Thread(target=radar_thread, daemon=True).start()
threading.Thread(target=imu_thread, daemon=True).start()

# Keep main alive until threads finish
import time
while threading.active_count() > 1:
    time.sleep(0.1)


print("All threads finished. Exiting.")

# =========================
# CSV EXPORT (DEBUG / VALIDATION)
# =========================
print("Writing CSV outputs...")

# ---- Radar Dynamic Points ----
with open("radar_dynamic_points.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["frame", "range", "angle", "elev", "doppler"])

    for i, frame in enumerate(frames):
        for pt in frame["dynamic_points"]:
            writer.writerow([
                i,
                pt["range"],
                pt["angle"],
                pt["elev"],
                pt["doppler"]
            ])

# ---- IMU Data ----
with open("imu_data.csv", "w", newline="") as f:
    writer = csv.writer(f)

    print(f"IMU entries collected: {len(imu_data_list)}")

    if len(imu_data_list) > 0:
        writer.writerow(list(imu_data_list[0].keys()))

        for row in imu_data_list:
            writer.writerow(list(row.values()))
    else:
        print("WARNING: IMU data list is empty at export time")

print("CSV export complete.")
