import serial
import time
import re
from datetime import datetime

# ================= 配置区域 =================
# 修改 COM
SERIAL_PORT = 'COM7'  
# 波特率
BAUD_RATE = 921600    
# 保存的文件名
OUTPUT_FILE = 'breathing_debug_log.csv'
# ===========================================

def extract_breathing_rate(log_line):
    """
    从日志行中提取呼吸率数值。
    """
    match = re.search(r"Breathing rate.*?(\d+(\.\d+)?)", log_line, re.IGNORECASE)
    if match:
        return match.group(1)
    return None

def main():
    try:
        # 打开串口
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"成功打开串口 {SERIAL_PORT} @ {BAUD_RATE}")
        print(f"正在监听数据 (呼吸率 + 调试信息)...")
        print(f"结果将保存到 {OUTPUT_FILE}")
        print("按 Ctrl+C 停止脚本\n")

        # 以追加模式打开文件
        with open(OUTPUT_FILE, "a", encoding="utf-8") as f:
            # 写入表头 (如果文件为空)
            if f.tell() == 0:
                f.write("Timestamp,Log_Type,Value_or_Info,Raw_Content\n")

            while True:
                if ser.in_waiting:
                    # 读取一行并解码
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                    except:
                        continue

                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    save_str = None
                    console_msg = None

                    # ---------------------------------------------------------
                    # 1. 捕获呼吸率 (Breathing Rate)
                    # ---------------------------------------------------------
                    if "Breathing rate" in line:
                        rate_value = extract_breathing_rate(line)
                        if rate_value:
                            save_str = f"{timestamp},RATE,{rate_value},{line}"
                            console_msg = f"[{timestamp}] 🔵 呼吸率: {rate_value} BPM"

                    # ---------------------------------------------------------
                    # 2. 捕获调试信息 ([DEBUG])
                    # ---------------------------------------------------------
                    elif "[DEBUG]" in line:
                        # 简单的分类，方便后续在 Excel 筛选
                        if "Raw Features" in line:
                            info_type = "DEBUG_RAW_FEAT"
                        elif "Norm Features" in line:
                            info_type = "DEBUG_NORM_FEAT"
                        elif "SVM Raw Result" in line:
                            info_type = "DEBUG_SVM_CALC"
                        else:
                            info_type = "DEBUG_OTHER"
                        
                        # 把整行内容作为 Info 存下来
                        clean_content = line.replace(",", ";") # 防止破坏 CSV 格式
                        save_str = f"{timestamp},{info_type},N/A,{clean_content}"
                        console_msg = f"[{timestamp}] 🟠 调试: {line}"

                    # ---------------------------------------------------------
                    # 3. 保存与打印
                    # ---------------------------------------------------------
                    if save_str:
                        f.write(save_str + "\n")
                        f.flush()
                    
                    if console_msg:
                        print(console_msg)

    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("请检查 COM 口是否被占用。")
    except KeyboardInterrupt:
        print("\n程序已停止。")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()