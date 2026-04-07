
import struct
import threading

# SERIAL CONFIG

IMU_PORT   = '/dev/tty.usbserial-IMU'
IMU_BAUD   = 115200

imu_data_list = []

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
threading.Thread(target=imu_thread, daemon=True).start()

# Keep main alive until threads finish
import time
while threading.active_count() > 1:
    time.sleep(0.1)


print("All threads finished. Exiting.")
