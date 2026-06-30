#!/usr/bin/env python3
"""
mcap_to_pointcloud.py
─────────────────────
Converts an MCAP file containing LiDAR data (LaserScan or PointCloud2)
into PLY / XYZ point cloud files suitable for CloudCompare, plus a
matplotlib visualization.

Supports:
  • sensor_msgs/msg/LaserScan   → 2D scan (or pseudo-3D stack of frames)
  • sensor_msgs/msg/PointCloud2 → full 3D point cloud

No ROS 2 installation required.
"""

import os
import sys
import math
import struct
import time
import glob
from pathlib import Path

# ─────────────────────────────────────────────
# Dependency check / friendly error messages
# ─────────────────────────────────────────────
MISSING = []
try:
    import numpy as np
except ImportError:
    MISSING.append("numpy")

try:
    from mcap.reader import make_reader
except ImportError:
    MISSING.append("mcap")

try:
    from mcap_ros2.decoder import DecoderFactory
    HAS_ROS2_DECODER = True
except ImportError:
    HAS_ROS2_DECODER = False   # will fall back to raw CDR parsing

try:
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[WARN] matplotlib not found – skipping visualizations.")

if MISSING:
    print("\n[ERROR] Missing required packages:", ", ".join(MISSING))
    print("Install them with:\n")
    print(f"  pip install {' '.join(MISSING)}")
    sys.exit(1)


# ═══════════════════════════════════════════════════════════════════════════════
# FILE SELECTION
# ═══════════════════════════════════════════════════════════════════════════════

def select_mcap_file() -> str:
    """Scan current directory for .mcap files and let user pick one."""
    files = sorted(glob.glob("*.mcap"))
    if not files:
        print("[ERROR] No .mcap files found in the current directory.")
        print("        Please run this script from the folder containing your .mcap files.")
        sys.exit(1)

    print("\n╔══════════════════════════════════════╗")
    print("║      MCAP → Point Cloud Converter    ║")
    print("╚══════════════════════════════════════╝\n")
    print(f"Found {len(files)} MCAP file(s):\n")
    for i, f in enumerate(files):
        size_mb = os.path.getsize(f) / 1_048_576
        print(f"  [{i + 1}] {f}  ({size_mb:.2f} MB)")

    while True:
        try:
            choice = input(f"\nEnter file number [1–{len(files)}]: ").strip()
            idx = int(choice) - 1
            if 0 <= idx < len(files):
                print(f"\n✔ Selected: {files[idx]}\n")
                return files[idx]
            else:
                print(f"    Please enter a number between 1 and {len(files)}.")
        except (ValueError, EOFError):
            print("    Invalid input. Please enter a number.")


# ═══════════════════════════════════════════════════════════════════════════════
# MCAP INSPECTION
# ═══════════════════════════════════════════════════════════════════════════════

def inspect_mcap(filepath: str) -> dict:
    """
    Open the MCAP file and collect:
      - all topic names and their schema/message types
      - message count per topic
      - time range
    Returns a dict keyed by topic name.
    """
    print("─" * 55)
    print("STEP 1 – Inspecting MCAP file structure …")
    print("─" * 55)

    topics = {}   # topic_name → {schema, count, first_ts, last_ts}

    with open(filepath, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()

        if summary:
            for ch in summary.channels.values():
                schema = summary.schemas.get(ch.schema_id)
                schema_name = schema.name if schema else "unknown"
                stats = summary.statistics
                count = 0
                if stats and stats.channel_message_counts:
                    count = stats.channel_message_counts.get(ch.id, 0)
                topics[ch.topic] = {
                    "schema": schema_name,
                    "count": count,
                    "channel_id": ch.id,
                }
        else:
            # Fall back: scan all messages
            for schema, channel, message in reader.iter_messages():
                t = channel.topic
                if t not in topics:
                    topics[t] = {
                        "schema": schema.name if schema else "unknown",
                        "count": 0,
                        "channel_id": channel.id,
                    }
                topics[t]["count"] += 1

    print(f"\n  Found {len(topics)} topic(s):\n")
    for t, info in topics.items():
        print(f"    {t}")
        print(f"      schema : {info['schema']}")
        print(f"      messages: {info['count']}")
    print()
    return topics


# ═══════════════════════════════════════════════════════════════════════════════
# TOPIC SELECTION
# ═══════════════════════════════════════════════════════════════════════════════

LASER_SCAN_SCHEMAS   = {"sensor_msgs/msg/LaserScan", "sensor_msgs/LaserScan"}
POINTCLOUD2_SCHEMAS  = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2"}

# Preferred topic name — script will pick this first if present
PREFERRED_PC2_TOPIC = "/map_3d"

def pick_lidar_topic(topics: dict) -> tuple[str, str]:
    """
    Auto-detect the best LiDAR topic.
    Priority: /map_3d (PointCloud2) > any PointCloud2 > LaserScan > user choice.
    Returns (topic_name, schema_name).
    """
    print("─" * 55)
    print("STEP 2 – Selecting LiDAR topic …")
    print("─" * 55)

    pc2_topics   = {t: v for t, v in topics.items()
                    if v["schema"] in POINTCLOUD2_SCHEMAS}
    laser_topics = {t: v for t, v in topics.items()
                    if v["schema"] in LASER_SCAN_SCHEMAS}

    # 1. Prefer /map_3d specifically
    if PREFERRED_PC2_TOPIC in pc2_topics:
        info = pc2_topics[PREFERRED_PC2_TOPIC]
        print(f"\n  ✔ Auto-selected PointCloud2 topic: {PREFERRED_PC2_TOPIC}")
        print(f"    ({info['count']} frames)\n")
        return PREFERRED_PC2_TOPIC, info["schema"]

    # 2. Any other PointCloud2 topic
    if pc2_topics:
        topic, info = next(iter(pc2_topics.items()))
        print(f"\n  ✔ Auto-selected PointCloud2 topic: {topic}")
        print(f"    ({info['count']} frames)\n")
        return topic, info["schema"]

    # 3. Fall back to LaserScan
    if laser_topics:
        topic, info = next(iter(laser_topics.items()))
        print(f"\n  ✔ Auto-selected LaserScan topic: {topic}")
        print(f"    ({info['count']} frames)\n")
        return topic, info["schema"]

    # No known type → let user choose
    print("\n  No recognised LiDAR schema found automatically.")
    topic_list = list(topics.items())
    for i, (t, v) in enumerate(topic_list):
        print(f"  [{i + 1}] {t}  [{v['schema']}]  ({v['count']} msgs)")

    while True:
        try:
            choice = int(input("\n  Enter topic number to try: ").strip()) - 1
            if 0 <= choice < len(topic_list):
                topic, info = topic_list[choice]
                return topic, info["schema"]
        except (ValueError, EOFError):
            pass
        print("  Invalid choice.")


# ═══════════════════════════════════════════════════════════════════════════════
# CDR RAW PARSER  (used when mcap_ros2 is not installed)
# ═══════════════════════════════════════════════════════════════════════════════

def _unpack(fmt, data, offset):
    size = struct.calcsize(fmt)
    return struct.unpack_from(fmt, data, offset), offset + size

def parse_laserscan_cdr(raw: bytes) -> dict | None:
    """
    Minimal CDR deserialiser for sensor_msgs/msg/LaserScan.

    Wire layout (little-endian CDR, 4-byte header):
      [0..3]   encapsulation header (0x00 0x01 0x00 0x00)
      Header:
        uint32  stamp.sec
        uint32  stamp.nanosec
        string  frame_id  (uint32 len + bytes + padding)
      float32  angle_min
      float32  angle_max
      float32  angle_increment
      float32  time_increment
      float32  scan_time
      float32  range_min
      float32  range_max
      float32[]  ranges   (uint32 count + floats)
      float32[]  intensities
    """
    try:
        offset = 4   # skip encapsulation header

        # Header stamp
        (sec, nsec), offset = _unpack("<II", raw, offset)

        # frame_id string
        (str_len,), offset = _unpack("<I", raw, offset)
        frame_id = raw[offset:offset + str_len - 1].decode("utf-8", errors="replace")
        offset += str_len
        # CDR string padding to 4-byte boundary
        offset = (offset + 3) & ~3

        # Scan parameters
        (angle_min,),       offset = _unpack("<f", raw, offset)
        (angle_max,),       offset = _unpack("<f", raw, offset)
        (angle_increment,), offset = _unpack("<f", raw, offset)
        (time_increment,),  offset = _unpack("<f", raw, offset)
        (scan_time,),       offset = _unpack("<f", raw, offset)
        (range_min,),       offset = _unpack("<f", raw, offset)
        (range_max,),       offset = _unpack("<f", raw, offset)

        # ranges array
        (n_ranges,), offset = _unpack("<I", raw, offset)
        ranges = list(struct.unpack_from(f"<{n_ranges}f", raw, offset))
        offset += n_ranges * 4

        # intensities array
        (n_int,), offset = _unpack("<I", raw, offset)
        intensities = list(struct.unpack_from(f"<{n_int}f", raw, offset))

        return {
            "stamp_sec": sec,
            "stamp_nsec": nsec,
            "frame_id": frame_id,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "range_min": range_min,
            "range_max": range_max,
            "ranges": ranges,
            "intensities": intensities,
        }
    except Exception as e:
        return None


def parse_pointcloud2_cdr(raw: bytes) -> dict | None:
    """
    Minimal CDR deserialiser for sensor_msgs/msg/PointCloud2.
    Extracts field layout and binary data blob.
    """
    try:
        offset = 4  # skip encapsulation header

        # Header
        (sec, nsec), offset = _unpack("<II", raw, offset)
        (str_len,),  offset = _unpack("<I", raw, offset)
        frame_id = raw[offset:offset + str_len - 1].decode("utf-8", errors="replace")
        offset += str_len
        offset = (offset + 3) & ~3

        (height,),   offset = _unpack("<I", raw, offset)
        (width,),    offset = _unpack("<I", raw, offset)

        # Fields array
        (n_fields,), offset = _unpack("<I", raw, offset)
        fields = []
        for _ in range(n_fields):
            (flen,), offset = _unpack("<I", raw, offset)
            fname = raw[offset:offset + flen - 1].decode("utf-8", errors="replace")
            offset += flen
            offset = (offset + 3) & ~3
            (foffset, datatype, count), offset = _unpack("<III", raw, offset)
            fields.append({"name": fname, "offset": foffset,
                           "datatype": datatype, "count": count})

        (is_bigendian,), offset = _unpack("<B", raw, offset)
        offset = (offset + 3) & ~3
        (point_step,),   offset = _unpack("<I", raw, offset)
        (row_step,),     offset = _unpack("<I", raw, offset)

        (data_len,),  offset = _unpack("<I", raw, offset)
        data_blob = raw[offset:offset + data_len]

        return {
            "height": height,
            "width": width,
            "fields": fields,
            "point_step": point_step,
            "row_step": row_step,
            "data": data_blob,
            "is_bigendian": is_bigendian,
        }
    except Exception:
        return None


DATATYPE_FMT = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}

def extract_pc2_xyz(pc2: dict) -> np.ndarray:
    """Extract XYZ points from a parsed PointCloud2 dict."""
    fields = {f["name"]: f for f in pc2["fields"]}
    step   = pc2["point_step"]
    data   = pc2["data"]
    n_pts  = len(data) // step
    endian = ">" if pc2["is_bigendian"] else "<"

    points = []
    for i in range(n_pts):
        base = i * step
        try:
            x = struct.unpack_from(endian + DATATYPE_FMT[fields["x"]["datatype"]],
                                   data, base + fields["x"]["offset"])[0]
            y = struct.unpack_from(endian + DATATYPE_FMT[fields["y"]["datatype"]],
                                   data, base + fields["y"]["offset"])[0]
            z_field = fields.get("z")
            z = struct.unpack_from(endian + DATATYPE_FMT[z_field["datatype"]],
                                   data, base + z_field["offset"])[0] if z_field else 0.0
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
        except Exception:
            continue
    return np.array(points, dtype=np.float32) if points else np.empty((0, 3), np.float32)


# ═══════════════════════════════════════════════════════════════════════════════
# FRAME EXTRACTION
# ═══════════════════════════════════════════════════════════════════════════════

def extract_laserscan_frames(filepath: str, topic: str) -> list[dict]:
    """Read all LaserScan messages from the chosen topic."""
    print("─" * 55)
    print("STEP 3 – Extracting LaserScan frames …")
    print("─" * 55)

    frames = []

    with open(filepath, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()] if HAS_ROS2_DECODER else [])

        for schema, channel, message in reader.iter_messages(topics=[topic]):
            if HAS_ROS2_DECODER and hasattr(message, "ranges"):
                # Decoded by mcap_ros2
                frame = {
                    "stamp_sec":  message.header.stamp.sec,
                    "stamp_nsec": message.header.stamp.nanosec,
                    "frame_id":   message.header.frame_id,
                    "angle_min":  message.angle_min,
                    "angle_max":  message.angle_max,
                    "angle_increment": message.angle_increment,
                    "range_min":  message.range_min,
                    "range_max":  message.range_max,
                    "ranges":     list(message.ranges),
                    "intensities": list(message.intensities) if message.intensities else [],
                }
            else:
                frame = parse_laserscan_cdr(message.data)

            if frame:
                frame["log_time"] = message.log_time   # nanoseconds
                frames.append(frame)

    print(f"\n  Extracted {len(frames)} frame(s) from '{topic}'\n")
    if frames:
        f0 = frames[0]
        print(f"  First frame metadata:")
        print(f"    frame_id      : {f0.get('frame_id', 'n/a')}")
        print(f"    angle_min     : {math.degrees(f0['angle_min']):.2f}°")
        print(f"    angle_max     : {math.degrees(f0['angle_max']):.2f}°")
        print(f"    angle_increment: {math.degrees(f0['angle_increment']):.4f}°")
        print(f"    range_min     : {f0['range_min']:.3f} m")
        print(f"    range_max     : {f0['range_max']:.3f} m")
        print(f"    n_ranges      : {len(f0['ranges'])}")
        print(f"    n_intensities : {len(f0.get('intensities', []))}")
        print()
    return frames


def extract_pointcloud2_frames(filepath: str, topic: str) -> list[np.ndarray]:
    """Read all PointCloud2 messages and return list of Nx3 arrays."""
    print("─" * 55)
    print("STEP 3 – Extracting PointCloud2 frames …")
    print("─" * 55)

    all_points = []

    with open(filepath, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()] if HAS_ROS2_DECODER else [])
        for schema, channel, message in reader.iter_messages(topics=[topic]):
            if HAS_ROS2_DECODER and hasattr(message, "fields"):
                # Build a simple dict to reuse extract_pc2_xyz
                pc2 = {
                    "height": message.height,
                    "width": message.width,
                    "fields": [{"name": fld.name, "offset": fld.offset,
                                 "datatype": fld.datatype, "count": fld.count}
                                for fld in message.fields],
                    "point_step": message.point_step,
                    "row_step": message.row_step,
                    "data": bytes(message.data),
                    "is_bigendian": message.is_bigendian,
                }
            else:
                pc2 = parse_pointcloud2_cdr(message.data)

            if pc2 is not None:
                pts = extract_pc2_xyz(pc2)
                if len(pts):
                    all_points.append(pts)

    print(f"\n  Extracted {len(all_points)} PointCloud2 frame(s) from '{topic}'\n")
    return all_points


# ═══════════════════════════════════════════════════════════════════════════════
# CONVERSION: LaserScan → XYZ points
# ═══════════════════════════════════════════════════════════════════════════════

def laserscan_to_points(frame: dict, z_offset: float = 0.0) -> np.ndarray:
    """
    Convert a single LaserScan frame to Cartesian XYZ points.
    z_offset: used for pseudo-3D stacking (metres or arbitrary index).
    Returns Nx3 float32 array.  NaN, inf, out-of-range values are dropped.
    """
    ranges     = np.array(frame["ranges"], dtype=np.float64)
    angle_min  = frame["angle_min"]
    angle_inc  = frame["angle_increment"]
    rmin       = frame["range_min"]
    rmax       = frame["range_max"]

    n = len(ranges)
    angles = angle_min + np.arange(n) * angle_inc

    # Filter invalid readings
    mask = (
        np.isfinite(ranges) &
        (ranges > rmin) &
        (ranges < rmax) &
        (ranges > 0.0)
    )
    r = ranges[mask]
    a = angles[mask]

    x = r * np.cos(a)
    y = r * np.sin(a)
    z = np.full_like(x, z_offset)

    return np.column_stack([x, y, z]).astype(np.float32)


def frames_to_pointcloud(frames: list[dict], strategy: str = "stack") -> np.ndarray:
    """
    Convert a list of LaserScan frames to a single Nx3 point array.

    strategy:
      'single' – only use first frame (truly 2D output, Z=0)
      'stack'  – use all frames, Z = normalised timestamp (pseudo-3D)
      'index'  – use all frames, Z = frame index (simpler pseudo-3D)
    """
    if not frames:
        return np.empty((0, 3), np.float32)

    print("─" * 55)
    print("STEP 4 – Converting to Cartesian XYZ …")
    print("─" * 55)

    if strategy == "single" or len(frames) == 1:
        pts = laserscan_to_points(frames[0], z_offset=0.0)
        print(f"\n  Strategy : single frame (2D, Z=0)")
        print(f"  Points   : {len(pts)}\n")
        return pts

    # Multi-frame
    t0 = frames[0]["log_time"]
    t1 = frames[-1]["log_time"]
    t_range = max(t1 - t0, 1)   # nanoseconds

    all_pts = []
    for i, frame in enumerate(frames):
        if strategy == "index":
            z = float(i) * 0.05   # 5 cm per frame — visually pleasant
        else:
            z = (frame["log_time"] - t0) / t_range * len(frames) * 0.05

        pts = laserscan_to_points(frame, z_offset=z)
        all_pts.append(pts)

    combined = np.vstack(all_pts)
    print(f"\n  Strategy  : {strategy} ({len(frames)} frames)")
    print(f"  Points    : {len(combined)}")
    print(f"  Z range   : {combined[:, 2].min():.3f} – {combined[:, 2].max():.3f} m\n")
    return combined


# ═══════════════════════════════════════════════════════════════════════════════
# EXPORT: PLY
# ═══════════════════════════════════════════════════════════════════════════════

def export_ply(points: np.ndarray, filepath: str):
    """Write a binary little-endian PLY file.  Colour encodes Z height."""
    print(f"  Writing PLY → {filepath}")
    n = len(points)

    # Normalise Z to 0-255 for colour
    zmin, zmax = points[:, 2].min(), points[:, 2].max()
    if zmax > zmin:
        z_norm = (points[:, 2] - zmin) / (zmax - zmin)
    else:
        z_norm = np.zeros(n)

    # Viridis-like colour: blue→green→yellow→red via HSV approximation
    # Simple approach: use matplotlib colormap if available, else greyscale
    if HAS_MPL:
        import matplotlib
        cmap = matplotlib.colormaps["viridis"]
        colours = (cmap(z_norm)[:, :3] * 255).astype(np.uint8)
    else:
        grey = (z_norm * 255).astype(np.uint8)
        colours = np.column_stack([grey, grey, grey])

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment Generated by mcap_to_pointcloud.py\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")

    with open(filepath, "wb") as f:
        f.write(header)
        for i in range(n):
            f.write(struct.pack("<fff", *points[i]))
            f.write(struct.pack("<BBB", *colours[i]))

    size_kb = os.path.getsize(filepath) / 1024
    print(f"  ✔ {n:,} points  ({size_kb:.1f} KB)")


# ═══════════════════════════════════════════════════════════════════════════════
# EXPORT: XYZ (fallback / CloudCompare ASCII)
# ═══════════════════════════════════════════════════════════════════════════════

def export_xyz(points: np.ndarray, filepath: str):
    """Write space-delimited XYZ file (CloudCompare ASCII import)."""
    print(f"  Writing XYZ → {filepath}")
    np.savetxt(filepath, points, fmt="%.6f", delimiter=" ")
    size_kb = os.path.getsize(filepath) / 1024
    print(f"  ✔ {len(points):,} points  ({size_kb:.1f} KB)")


# ═══════════════════════════════════════════════════════════════════════════════
# VISUALISATION
# ═══════════════════════════════════════════════════════════════════════════════

def visualise(points: np.ndarray, title: str, output_png: str):
    if not HAS_MPL:
        return

    fig = plt.figure(figsize=(14, 6))
    fig.suptitle(title, fontsize=13, fontweight="bold")

    is_3d = points[:, 2].ptp() > 0.01   # more than 1 cm Z spread

    if is_3d:
        # Left: top-down (XY)
        ax1 = fig.add_subplot(1, 2, 1)
        sc = ax1.scatter(points[:, 0], points[:, 1],
                         c=points[:, 2], cmap="viridis",
                         s=0.5, linewidths=0)
        plt.colorbar(sc, ax=ax1, label="Z / height (m)")
        ax1.set_title("Top-down view (XY plane)")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_aspect("equal")
        ax1.grid(True, alpha=0.3)

        # Right: side view (XZ)
        ax2 = fig.add_subplot(1, 2, 2)
        ax2.scatter(points[:, 0], points[:, 2],
                    c=points[:, 2], cmap="plasma",
                    s=0.5, linewidths=0)
        ax2.set_title("Side view (XZ plane)")
        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Z / frame stack (m)")
        ax2.grid(True, alpha=0.3)
    else:
        # Single 2D scan
        ax = fig.add_subplot(1, 1, 1)
        ax.scatter(points[:, 0], points[:, 1],
                   s=1.5, c="steelblue", linewidths=0)
        ax.set_title("2D LiDAR scan (XY plane)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        # Mark origin (sensor position)
        ax.plot(0, 0, "r+", markersize=10, markeredgewidth=2, label="Sensor origin")
        ax.legend()

    plt.tight_layout()
    plt.savefig(output_png, dpi=150, bbox_inches="tight")
    print(f"  ✔ Saved visualisation → {output_png}")
    plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    # 1. File selection
    mcap_path = select_mcap_file()
    stem = Path(mcap_path).stem

    # 2. Inspect
    topics = inspect_mcap(mcap_path)

    # 3. Pick topic
    topic, schema = pick_lidar_topic(topics)

    # 4. Extract & convert
    # PointCloud2 checked first — /map_3d is the primary target
    if schema in POINTCLOUD2_SCHEMAS or "PointCloud2" in schema:
        frame_arrays = extract_pointcloud2_frames(mcap_path, topic)
        if not frame_arrays:
            print("[ERROR] No valid PointCloud2 frames found.")
            sys.exit(1)
        points = np.vstack(frame_arrays)
        print(f"  Combined {len(frame_arrays)} PointCloud2 frame(s) → {len(points):,} total points\n")

    elif schema in LASER_SCAN_SCHEMAS or "LaserScan" in schema:
        frames = extract_laserscan_frames(mcap_path, topic)
        if not frames:
            print("[ERROR] No valid LaserScan frames found.")
            sys.exit(1)

        # Decide strategy
        if len(frames) == 1:
            strategy = "single"
            print("  ℹ Single frame detected → 2D output (Z=0).")
            print("    This is a flat 2D scan.  CloudCompare will display it in the XY plane.\n")
        else:
            print(f"  ℹ {len(frames)} frames detected.")
            print("  Stacking frames along Z axis for pseudo-3D reconstruction.")
            print("  Z = frame_index × 0.05 m  (5 cm virtual layer spacing)\n")
            strategy = "index"

        points = frames_to_pointcloud(frames, strategy=strategy)
    else:
        print(f"[WARN] Unknown schema '{schema}'.  Attempting raw CDR LaserScan parse …")
        frames = extract_laserscan_frames(mcap_path, topic)
        if not frames:
            print("[ERROR] Could not decode any messages.  Exiting.")
            sys.exit(1)
        points = frames_to_pointcloud(frames, strategy="index" if len(frames) > 1 else "single")

    if len(points) == 0:
        print("[ERROR] Zero valid points extracted.  Check file integrity.")
        sys.exit(1)

    # 5. Summary
    print("─" * 55)
    print("STEP 5 – Point cloud summary")
    print("─" * 55)
    print(f"  Total points : {len(points):,}")
    print(f"  X range      : {points[:,0].min():.3f} – {points[:,0].max():.3f} m")
    print(f"  Y range      : {points[:,1].min():.3f} – {points[:,1].max():.3f} m")
    print(f"  Z range      : {points[:,2].min():.3f} – {points[:,2].max():.3f} m\n")

    # 6. Export
    print("─" * 55)
    print("STEP 6 – Exporting files …")
    print("─" * 55)

    ply_out = f"{stem}_pointcloud.ply"
    xyz_out = f"{stem}_pointcloud.xyz"
    png_out = f"{stem}_scan_viz.png"

    export_ply(points, ply_out)
    export_xyz(points, xyz_out)

    # 7. Visualise
    print()
    print("─" * 55)
    print("STEP 7 – Generating visualisation …")
    print("─" * 55)
    visualise(points, title=f"{mcap_path} → {topic}", output_png=png_out)

    # 8. Final report
    print()
    print("═" * 55)
    print("  Done!  Output files:")
    print(f"    PLY  → {ply_out}   (preferred for CloudCompare)")
    print(f"    XYZ  → {xyz_out}   (ASCII fallback)")
    if HAS_MPL:
        print(f"    PNG  → {png_out}")
    print()
    print("  How to open in CloudCompare:")
    print("    1. Launch CloudCompare")
    print(f"    2. File → Open → select  {ply_out}")
    print("    3. It will auto-detect PLY format")
    print("    4. Use  Edit → Colors → Height Ramp  to colour by Z")
    print("    5. Use  Display → Toggle Viewer-Based Perspective  for 3D view")
    print("═" * 55)


if __name__ == "__main__":
    main()
