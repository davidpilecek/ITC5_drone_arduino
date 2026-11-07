"""Convert a Teensy ITC IMU binary log to CSV using only Python's standard library.

Usage: python imu_log_to_csv.py IMU0000.BIN output.csv
Incomplete trailing records are ignored with a warning. Existing output is never overwritten.
"""
import argparse
import csv
import struct
import sys
from pathlib import Path

HEADER = struct.Struct("<8sIIII")
RECORD = struct.Struct("<II12fII")


def convert(source: Path, destination: Path) -> int:
    with source.open("rb") as infile:
        header = infile.read(512)
        if len(header) != 512:
            raise ValueError("File has no complete 512-byte ITC header")
        magic, version, record_bytes, period_us, capacity = HEADER.unpack_from(header)
        if magic != b"ITCIMU1\0" or version != 1 or record_bytes != RECORD.size:
            raise ValueError("Unsupported IMU log format")
        count = 0
        elapsed_us = 0
        previous_timestamp = None
        with destination.open("x", newline="", encoding="utf-8") as outfile:
            writer = csv.writer(outfile)
            writer.writerow([
                "sequence", "timestamp_us", "elapsed_s",
                "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
                "mx_uT", "my_uT", "mz_uT", "roll_deg", "pitch_deg", "yaw_deg",
                "mag_valid", "mag_calibrated", "attitude_valid",
            ])
            while True:
                data = infile.read(RECORD.size)
                if not data:
                    break
                if len(data) != RECORD.size:
                    print("Warning: incomplete final record ignored", file=sys.stderr)
                    break
                timestamp, sequence, *fields = RECORD.unpack(data)
                values, flags, _reserved = fields[:12], fields[12], fields[13]
                if not flags & 1:  # Padding in a partial sector.
                    continue
                if previous_timestamp is not None:
                    elapsed_us += (timestamp - previous_timestamp) & 0xffffffff
                previous_timestamp = timestamp
                writer.writerow([sequence, timestamp, elapsed_us * 1e-6, *values,
                                 int(bool(flags & 2)), int(bool(flags & 4)), int(bool(flags & 8))])
                count += 1
    timing = "event-driven (use timestamps)" if period_us == 0 else f"configured period {period_us} us"
    print(f"Converted {count} records; {timing}, buffer {capacity} samples")
    return count


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        convert(args.input, args.output)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Error: {error}\n")
