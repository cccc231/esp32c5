import serial
import time

# 配置串口 (根据实际情况修改)
serial_port = 'COM7'  
baud_rate = 921600   

def parse_csi_line(line):
    """
    解析格式: CSI_DATA,10,...,"[17,7,15...]"
    """
    line = line.strip()
    
    # 1. 基本校验
    if not line.startswith("CSI_DATA"):
        return None
        
    try:
        # 2. 关键步骤：寻找方括号的位置
        start_idx = line.find("[")
        end_idx = line.find("]")
        
        if start_idx == -1 or end_idx == -1:
            return None
            
        # 3. 提取方括号内的纯数字字符串
        # line[start_idx+1 : end_idx] 得到 "17,7,15,12..."
        raw_numbers_str = line[start_idx+1 : end_idx]
        
        # 4. 分割并转换为整数列表
        if not raw_numbers_str:
            return []
            
        csi_data = [int(x) for x in raw_numbers_str.split(',')]
        
        return csi_data

    except Exception as e:
        print(f"解析错误: {e}")
        return None

# --- 主程序 ---
ser = serial.Serial(serial_port, baud_rate, timeout=1)
output_filename = "csi_data_log.txt"
# 以追加模式打开文件
file_handle = open(output_filename, "a", encoding="utf-8")

print(f"开始监听 {serial_port}... 数据将保存到 {output_filename}")

try:
    while True:
        if ser.in_waiting:
            # 读取一行原始数据
            raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # 如果是 CSI 数据行
            if "CSI_DATA" in raw_line:
                # 1. 解析数据
                csi_values = parse_csi_line(raw_line)
                
                if csi_values:
                    # 2. 可以在这里做实时处理 (比如检查长度)
                    # print(f"收到包: 长度={len(csi_values)}")
                    
                    # 3. 【重要】保存到 txt 文件
                    # 为了方便后续 MATLAB/Python 分析，我们只保存纯数字部分
                    # 将列表转换为逗号分隔的字符串: "17,7,15,12..."
                    save_str = ",".join(map(str, csi_values))
                    
                    file_handle.write(save_str + "\n")
                    file_handle.flush() # 立即写入硬盘
                    
                    print(f"已保存: {save_str[:30]}... (共{len(csi_values)}个数据)")

except KeyboardInterrupt:
    print("\n停止监听")
    ser.close()
    file_handle.close()