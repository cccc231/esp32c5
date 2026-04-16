import serial
import time
import argparse
import re
from datetime import datetime

# ================= 配置 =================
DEFAULT_PORT = "COM7"   # <--- 修改为你的端口
BAUD_RATE = 921600      # 确保与 idf.py monitor 一致
# =======================================

def start_logging(port, baud):
    # 文件名带时间戳
    filename = f"amplitude_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"--- Amplitude Logger ---")
    print(f"Target: RAW_AMP only")
    print(f"Saving to: {filename}")
    print(f"Connecting to {port}...")

    # 匹配 RAW_AMP:123
    regex = re.compile(r"RAW_AMP:([0-9.]+)")

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"Serial Error: {e}")
        return

    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("timestamp,amplitude\n") # 表头
            
            count = 0
            print("Logging... Press Ctrl+C to stop.")

            while True:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    # 只处理 RAW_AMP
                    if line.startswith("RAW_AMP"):
                        match = regex.search(line)
                        if match:
                            val = match.group(1)
                            ts = time.time()
                            f.write(f"{ts},{val}\n")
                            count += 1
                            
                            if count % 1000 == 0:
                                print(f"\rCaptured {count} samples...", end="")
                
                except Exception:
                    continue

    except KeyboardInterrupt:
        print(f"\nStopped. Total samples saved: {count}")
    finally:
        ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str, default=DEFAULT_PORT)
    args = parser.parse_args()
    start_logging(args.port, BAUD_RATE)