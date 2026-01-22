import sys
import csv
import math
import os

# --- 物理定数の設定 ---
PULLEY_RADIUS = 0.012  # 12mm
RESOLUTION = 4096      # XL430分解能
CENTER_POS = 2048      # 初期位置（ニュートラル）
K = RESOLUTION / (2 * math.pi * PULLEY_RADIUS)

def apply_zero_limit_rule(val):
    # 上限4095または下限0を超えたら0にする絶対ルール
    if val > 4095 or val < 0:
        return 0
    return int(round(val))

def main():
    # 引数（CSVのパス）のチェック
    if len(sys.argv) < 2:
        print("使用法: python3 import_csv.py [CSVファイルをドラッグ＆ドロップ]")
        return

    csv_path = sys.argv[1]
    output_header = "motion_data.h"

    if not os.path.exists(csv_path):
        print(f"エラー: ファイルが見つかりません -> {csv_path}")
        return

    results = []

    # CSV読み込み（pandasを使わない標準機能版）
    with open(csv_path, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # joint1〜4を取得（単位: m）
            d1 = float(row['joint1'])
            d2 = float(row['joint2'])
            d3 = float(row['joint3'])
            d4 = float(row['joint4'])

            # 指定された絶対的マッピング・ルールに基づく計算
            # m1 = j2 - j1, m2 = j1 - j4, m3 = j4 - j3, m4 = j3 - j2
            m1 = apply_zero_limit_rule(CENTER_POS + (d2 - d1) * K)
            m2 = apply_zero_limit_rule(CENTER_POS + (d1 - d4) * K)
            m3 = apply_zero_limit_rule(CENTER_POS + (d4 - d3) * K)
            m4 = apply_zero_limit_rule(CENTER_POS + (d3 - d2) * K)

            results.append((m1, m2, m3, m4))

    # Arduino用ヘッダー書き出し
    with open(output_header, "w") as f:
        f.write("#ifndef MOTION_DATA_H\n#define MOTION_DATA_H\n\n")
        f.write(f"const int TOTAL_STEPS = {len(results)};\n")
        f.write("const uint16_t MOTION_DATA[][4] = {\n")
        for res in results:
            f.write(f"  {{{res[0]}, {res[1]}, {res[2]}, {res[3]}}},\n")
        f.write("};\n\n#endif")
    
    print(f"成功: {output_header} を作成しました。")
    print(f"対象ファイル: {os.path.basename(csv_path)}")

if __name__ == "__main__":
    main()