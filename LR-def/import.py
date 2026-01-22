import sys
import csv
import math
import os

PULLEY_RADIUS = 0.012
RESOLUTION = 4096
K = RESOLUTION / (2 * math.pi * PULLEY_RADIUS)

def calculate_step(v1, v2):
    # 異符号の時のみ、絶対値が大きい方を採用
    if v1 * v2 < 0:
        val = max(abs(v1), abs(v2)) * K
        if val > 4095: return 0
        return int(round(val))
    return 0

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 import.py [CSV_FILE_PATH]")
        return
    csv_path = sys.argv[1]
    results = []
    with open(csv_path, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            d1, d2, d3, d4 = float(row['joint1']), float(row['joint2']), float(row['joint3']), float(row['joint4'])
            results.append((calculate_step(d1, d2), calculate_step(d2, d3), calculate_step(d3, d4), calculate_step(d4, d1)))
    with open("motion_data.h", "w") as f:
        f.write(f"#ifndef MOTION_DATA_H\n#define MOTION_DATA_H\nconst int TOTAL_STEPS = {len(results)};\n")
        f.write("const uint16_t MOTION_DATA[][4] = {\n")
        for res in results: f.write(f"  {{{res[0]}, {res[1]}, {res[2]}, {res[3]}}},\n")
        f.write("};\n#endif")
    print("Done: motion_data.h created.")

if __name__ == "__main__":
    main()