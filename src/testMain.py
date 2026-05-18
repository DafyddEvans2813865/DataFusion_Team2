

import struct
import time
import serial

from radar.radar_parser_impl import RadarParser


# -----------------------------------------------------------------------------
# TI mmWave UART Configuration
# -----------------------------------------------------------------------------
CLI_PORT = '/dev/tty.usbserial-01755FF80'
DATA_PORT = '/dev/tty.usbserial-01755FF81'
CFG_FILE = '/Users/saulcallan/Documents/GitHub/DataFusion_Team2/src/Mobile_Tracker_6843_ISK.cfg'

CLI_BAUD = 115200
DATA_BAUD = 921600

MAGIC_WORD = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
HEADER_SIZE = 40
MAX_PACKET_SIZE = 2**18


# -----------------------------------------------------------------------------
# Send Radar Configuration
# -----------------------------------------------------------------------------
def send_config():

    print(f"Opening CLI port: {CLI_PORT}")

    cli = serial.Serial(CLI_PORT, CLI_BAUD, timeout=1)

    with open(CFG_FILE, 'r') as f:
        lines = f.readlines()

    print(f"Sending config: {CFG_FILE}")

    for line in lines:
        line = line.strip()

        if not line or line.startswith('%'):
            continue

        print(f"> {line}")

        cli.write((line + '\n').encode())
        time.sleep(0.05)

        response = cli.read(cli.in_waiting or 1)

        if response:
            try:
                print(response.decode(errors='ignore').strip())
            except:
                pass

    cli.close()

    print("Radar config sent successfully")


# -----------------------------------------------------------------------------
# Read One Complete UART Frame
# -----------------------------------------------------------------------------
def read_frame(ser, buffer: bytes):

    while True:

        n = ser.in_waiting

        if n > 0:
            buffer += ser.read(n)
        else:
            chunk = ser.read(64)

            if chunk:
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

        if total_len <= HEADER_SIZE or total_len > MAX_PACKET_SIZE:
            buffer = buffer[8:]
            continue

        while len(buffer) < total_len:
            chunk = ser.read(min(total_len - len(buffer), 8192))

            if not chunk:
                return None, buffer

            buffer += chunk

        packet = buffer[:total_len]
        buffer = buffer[total_len:]

        return packet, buffer


# -----------------------------------------------------------------------------
# Main Test
# -----------------------------------------------------------------------------
def main():

    print("=" * 60)
    print("TI mmWave Realtime Parser Test")
    print("=" * 60)

    try:
        send_config()

    except Exception as e:
        print(f"Failed to send config: {e}")
        return

    try:
        print(f"Opening DATA port: {DATA_PORT}")

        data_serial = serial.Serial(DATA_PORT, DATA_BAUD, timeout=5)
        data_serial.set_buffer_size(rx_size=2**17)

    except Exception as e:
        print(f"Failed to open data port: {e}")
        return

    parser = RadarParser()

    buffer = b''

    frame_count = 0
    point_count = 0
    parse_errors = 0

    print("\nReceiving radar data for 10 seconds...\n")

    start_time = time.time()

    try:
        while time.time() - start_time < 10:

            frame, buffer = read_frame(data_serial, buffer)

            if frame is None:
                continue

            try:
                before = len(parser.points)

                parser._parse_packet(frame)

                after = len(parser.points)

                new_points = after - before

                frame_count += 1
                point_count += new_points

                print(
                    f"Frame {frame_count:04d} | "
                    f"Packet Size: {len(frame):5d} bytes | "
                    f"Points: {new_points:3d}"
                )

            except Exception as e:
                parse_errors += 1
                print(f"Parse error: {e}")

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        data_serial.close()

    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)

    print(f"Frames received : {frame_count}")
    print(f"Points parsed   : {point_count}")
    print(f"Parse errors    : {parse_errors}")

    if frame_count > 0 and parse_errors == 0:
        print("\nSUCCESS: Radar data received and parsed correctly")
    elif frame_count > 0:
        print("\nWARNING: Data received but some packets failed parsing")
    else:
        print("\nFAILURE: No valid radar frames received")


if __name__ == '__main__':
    main()