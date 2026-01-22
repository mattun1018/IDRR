import sys
import csv
import math
import os

# ==========================================
# ユーザー設定（ここを調整してください）
# ==========================================

# 1. 排他制御（奇数ペアか偶数ペアの片方しか動かさないルール）を有効にするか
# False (デフォルト): 各モータは独立して動作（以前のルール）。「フラグがoff」の状態です。
# True: 常に奇数番(M1, M3)か偶数番(M2, M4)のどちらか一方のペアのみを動かす。
ENABLE_GAIT_EXCLUSION = False

# 2. 同値（タイ）になった時の挙動（ENABLE_GAIT_EXCLUSION = True の時のみ有効）
# "none" -> どちらも動かさない (0にする)。これがご要望の「デフォルトoff」に近い挙動です。
# "odd"  -> 奇数ペアを優先
# "even" -> 偶数ペアを優先
TIE_BREAK_MODE = "none" 

# 物理定数
PULLEY_RADIUS = 0.012
RESOLUTION = 4096
K = RESOLUTION / (2 * math.pi * PULLEY_RADIUS)

def get_potential_step(v1, v2):
    # 異符号の時のみ、絶対値が大きい方の変位を計算
    if v1 * v2 < 0:
        val = max(abs(v1), abs(v2)) * K
        return min(4095, int(round(val)))
    return 0

def main():
    if len(sys.argv) < 2:
        print("使用法: python3 import.py [CSVファイルパス]")
        return
    csv_path = sys.argv[1]
    results = []

    with open(csv_path, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            d1, d2, d3, d4 = float(row['joint1']), float(row['joint2']), float(row['joint3']), float(row['joint4'])
            
            # 各モータのポテンシャル（候補値）を計算
            m1_p = get_potential_step(d1, d2)
            m2_p = get_potential_step(d2, d3)
            m3_p = get_potential_step(d3, d4)
            m4_p = get_potential_step(d4, d1)
            
            if ENABLE_GAIT_EXCLUSION:
                # 【排他制御モード】
                odd_sum = m1_p + m3_p
                even_sum = m2_p + m4_p
                
                if odd_sum > even_sum:
                    selected = "odd"
                elif even_sum > odd_sum:
                    selected = "even"
                else:
                    # 合計が同じ（またはどちらも0）の場合は TIE_BREAK_MODE に従う
                    selected = TIE_BREAK_MODE 

                if selected == "odd":
                    m1, m2, m3, m4 = m1_p, 0, m3_p, 0
                elif selected == "even":
                    m1, m2, m3, m4 = 0, m2_p, 0, m4_p
                else:
                    m1, m2, m3, m4 = 0, 0, 0, 0
            else:
                # 【独立制御モード（デフォルトoff）】
                # 各モータは自分の担当ペアの符号だけを見て独立に動く
                m1, m2, m3, m4 = m1_p, m2_p, m3_p, m4_p
                
            results.append((m1, m2, m3, m4))

    # motion_data.h 書き出し
    with open("motion_data.h", "w") as f:
        f.write(f"#ifndef MOTION_DATA_H\n#define MOTION_DATA_H\nconst int TOTAL_STEPS = {len(results)};\n")
        f.write("const uint16_t MOTION_DATA[][4] = {\n")
        for res in results:
            f.write(f"  {{{res[0]}, {res[1]}, {res[2]}, {res[3]}}},\n")
        f.write("};\n#endif")
    
    status = "有効 (Conflict解消: " + TIE_BREAK_MODE + ")" if ENABLE_GAIT_EXCLUSION else "無効 (独立動作)"
    print(f"Done: motion_data.h created. 排他制御フラグ: {status}")

if __name__ == "__main__":
    main()