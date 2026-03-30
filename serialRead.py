import struct
import csv
import serial

def u32(data, i):
    return int.from_bytes(data[i:i+4], 'little')

def f32(data, i):
    return struct.unpack('<f', data[i:i+4])[0]
 # =========================
# SERIAL CONFIG
# =========================
PORT = 'COM9'   # change as needed (e.g. '/dev/ttyUSB0' on Mac/Linux)
BAUD = 921600

ser = serial.Serial(PORT, BAUD, timeout=1)

raw_bytes = bytearray()


 # Magic word in raw bytes
magic = bytes.fromhex('0201040306050807')

# Continuous buffer processing
buffer = bytearray()

frames = []

print("Listening on serial port...")

while True:
    buffer += ser.read(4096)

    while True:
        start = buffer.find(magic)
        if start == -1:
            break

        if len(buffer) < start + 16:
            break

        pkt_len = int.from_bytes(buffer[start+12:start+16], 'little')

        if len(buffer) < start + pkt_len:
            break

        p = buffer[start:start+pkt_len]

    # Correct offsets (after 8-byte magic word)
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

    # TLV length is payload only

    for tlv in range(header['numTLV']):
        if idx + 8 > len(p):
            break

        tlv_type   = u32(p, idx)
        tlv_length = u32(p, idx + 4)

        print("TLV:", tlv_type, "Length:", tlv_length)

        tlv_start = idx
        tlv_end   = tlv_start + 8 + tlv_length

        # Sanity check
        if tlv_length < 8 or tlv_end > len(p):
            print("Invalid TLV detected, stopping parse")
            break

        idx += 8   # move pointer to start of TLV payload

        if tlv_type == 1:  # Dynamic detected points

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

        elif tlv_type == 7:  # Side info (dynamic)
            while idx + 4 <= tlv_end:
                snr   = int.from_bytes(p[idx:idx+2], 'little')
                noise = int.from_bytes(p[idx+2:idx+4], 'little')

                frame_data["dynamic_sideinfo"].append({
                    "snr": snr,
                    "noise": noise
                })
                idx += 4
            idx = tlv_end

        elif tlv_type == 8:  # Static detected points
            while idx + 16 <= tlv_end:
                x = f32(p, idx)
                y = f32(p, idx + 4)
                z = f32(p, idx + 8)
                d = f32(p, idx + 12)

                frame_data["static_points"].append({
                    "x": x,
                    "y": y,
                    "z": z,
                    "doppler": d
                })
                idx += 16
            idx = tlv_end

        elif tlv_type == 9:  # Static side info
            while idx + 4 <= tlv_end:
                snr   = int.from_bytes(p[idx:idx+2], 'little')
                noise = int.from_bytes(p[idx+2:idx+4], 'little')

                frame_data["static_sideinfo"].append({
                    "snr": snr,
                    "noise": noise
                })
                idx += 4
            idx = tlv_end

        elif tlv_type == 10:  # Tracked objects
            while idx + 40 <= tlv_end:
                tid = u32(p, idx)
                px  = f32(p, idx + 4)
                py  = f32(p, idx + 8)
                vx  = f32(p, idx + 12)
                vy  = f32(p, idx + 16)
                ax  = f32(p, idx + 20)
                ay  = f32(p, idx + 24)
                pz  = f32(p, idx + 28)
                vz  = f32(p, idx + 32)
                az  = f32(p, idx + 36)

                frame_data["tracks"].append({
                    "id": tid,
                    "pos": (px, py, pz),
                    "vel": (vx, vy, vz),
                    "acc": (ax, ay, az)
                })
                idx += 40
            idx = tlv_end

        elif tlv_type == 11:  # Point to track association
            while idx < tlv_end:
                tid = p[idx]
                frame_data["associations"].append(tid)
                idx += 1
            idx = tlv_end

        else:
            print(f"[Unhandled TLV type {tlv_type}] skipping")
            idx = tlv_end
        frames.append(frame_data)
        print("Frame", header['frameNumber'], "parsed with", len(frame_data["dynamic_points"]), "points")

        buffer = buffer[start + pkt_len:]

        # Optional: print frame times continuously
        print("Frame time:", frame_data["header"]["time"])

# -------- CSV EXPORT --------

# Dynamic points CSV
with open("dynamic_points.csv", "w", newline="") as fcsv:
    writer = csv.writer(fcsv)
    writer.writerow(["frame", "range", "angle", "elev", "doppler", "snr", "noise"])

    for frame_idx, frame in enumerate(frames):
        dyn = frame["dynamic_points"]
        side = frame["dynamic_sideinfo"]

        for i in range(len(dyn)):
            snr = side[i]["snr"] if i < len(side) else ""
            noise = side[i]["noise"] if i < len(side) else ""

            writer.writerow([
                frame_idx,
                dyn[i]["range"],
                dyn[i]["angle"],
                dyn[i]["elev"],
                dyn[i]["doppler"],
                snr,
                noise
            ])

# Static points CSV
with open("static_points.csv", "w", newline="") as fcsv:
    writer = csv.writer(fcsv)
    writer.writerow(["frame", "x", "y", "z", "doppler", "snr", "noise"])

    for frame_idx, frame in enumerate(frames):
        stat = frame["static_points"]
        side = frame["static_sideinfo"]

        for i in range(len(stat)):
            snr = side[i]["snr"] if i < len(side) else ""
            noise = side[i]["noise"] if i < len(side) else ""

            writer.writerow([
                frame_idx,
                stat[i]["x"],
                stat[i]["y"],
                stat[i]["z"],
                stat[i]["doppler"],
                snr,
                noise
            ])

# Tracks CSV
with open("tracks.csv", "w", newline="") as fcsv:
    writer = csv.writer(fcsv)
    writer.writerow(["frame", "track_id", "px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az"])

    for frame_idx, frame in enumerate(frames):
        for t in frame["tracks"]:
            writer.writerow([
                frame_idx,
                t["id"],
                t["pos"][0],
                t["pos"][1],
                t["pos"][2],
                t["vel"][0],
                t["vel"][1],
                t["vel"][2],
                t["acc"][0],
                t["acc"][1],
                t["acc"][2]
            ])

# Associations CSV
with open("associations.csv", "w", newline="") as fcsv:
    writer = csv.writer(fcsv)
    writer.writerow(["frame", "target_id"])

    for frame_idx, frame in enumerate(frames):
        for tid in frame["associations"]:
            writer.writerow([frame_idx, tid])

print("CSV export complete")
# Print only time from each frame header
for i, frame in enumerate(frames):
    print(f"Frame {i} time:", frame["header"]["time"])
