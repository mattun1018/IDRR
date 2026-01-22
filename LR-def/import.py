import sys
import csv
import math
import os

# ==========================================
# 制御モード設定（ここを書き換えてください）
# ==========================================
# "off"         -> 排他制御なし。4つのモータがそれぞれ独立して動作（フラグ判定なし）
# "manual_odd"  -> 常に 奇数ペア(M1, M3) だけを動かす。偶数ペアは 2048 固定。
# "manual_even" -> 常に 偶数ペア(M2, M4) だけを動かす。奇数ペアは 2048 固定。
# "auto"        -> シミュレーションの変位が大きい方のペアを自動で選ぶ。
MODE = "manual_odd" 

# 物理定数
PULLEY_RADIUS = 0.016  # 16mm
RESOLUTION = 4096
CENTER_POS = 2048
K = RESOLUTION / (2 * math.pi * PULLEY_RADIUS)

def get_directional_step(v_a, v_b):
    """
    異符号判定と方向性決定のコアロジック
    - (-, +) パターン -> 巻き取り (> 2048)
    - (+, -) パターン -> 送り出し (< 2048)
    """
    if v_a * v_b < 0:
        mag = max(abs(v_a), abs(v_b))
        if v_a < 0 and v_b > 0:
            # 収縮・伸長パターン：巻き取り (> 2048)
            val = CENTER_POS + (mag * K)
        else:
            # 伸長・収縮パターン：送り出し (< 2048)
            val = CENTER_POS - (mag * K)
        return max(0, min(4095, int(round(val))))
    return CENTER_POS

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
            
            # 各モータの候補計算
            m1_p = get_directional_step(d1, d2)
            m2_p = get_directional_step(d2, d3)
            m3_p = get_directional_step(d3, d4)
            m4_p = get_directional_step(d4, d1)
            
            # --- モードに基づくペア選択 ---
            if MODE == "off":
                m1, m2, m3, m4 = m1_p, m2_p, m3_p, m4_p
            elif MODE == "manual_odd":
                m1, m2, m3, m4 = m1_p, CENTER_POS, m3_p, CENTER_POS
            elif MODE == "manual_even":
                m1, m2, m3, m4 = CENTER_POS, m2_p, CENTER_POS, m4_p
            elif MODE == "auto":
                odd_score = abs(m1_p - CENTER_POS) + abs(m3_p - CENTER_POS)
                even_score = abs(m2_p - CENTER_POS) + abs(m4_p - CENTER_POS)
                if odd_score >= even_score:
                    m1, m2, m3, m4 = m1_p, CENTER_POS, m3_p, CENTER_POS
                else:
                    m1, m2, m3, m4 = CENTER_POS, m2_p, CENTER_POS, m4_p
                
            results.append((m1, m2, m3, m4))

    # motion_data.h 書き出し
    with open("motion_data.h", "w") as f:
        f.write("#ifndef MOTION_DATA_H\n#define MOTION_DATA_H\n\n")
        f.write(f"const int TOTAL_STEPS = {len(results)};\n")
        f.write("const uint16_t MOTION_DATA[][4] = {\n")
        for res in results:
            f.write(f"  {{{res[0]}, {res[1]}, {res[2]}, {res[3]}}},\n")
        f.write("};\n\n#endif")
    
    print(f"Done: motion_data.h created. 設定モード: {MODE}")

if __name__ == "__main__":
    main()