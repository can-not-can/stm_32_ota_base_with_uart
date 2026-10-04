from serial import Serial, SerialException
from serial.tools import list_ports
import time
# ====================== 配置区，自行修改 ======================
COM_PORT = "COM4"        # 改成你的串口设备
BAUDRATE = 115200
CHUNK_SIZE = 128         # 固件分片大小，和boot fwBuf一致
REQ_HEAD_BYTE = 0x5A     # Boot->PC: 请求发送包头
READY_BYTE    = 0xA5     # Boot->PC: Flash擦除完成，请求发送固件
SUCCESS_BYTE  = 0x55     # Boot->PC: 校验通过，即将跳转新App
READY_TIMEOUT = 30       # 握手总超时(秒)，含用户按复位的等待时间
RESULT_TIMEOUT = 8       # 固件发完后等待Boot最终结果的超时(秒)
VERSION = 1              # 固件版本号 uint32
BIN_FILE_PATH = r"S:\stm32cubemx_project_1\rtos_hal\study\OTA_APP_Part\MDK-ARM\$BOTA_APP_Part.bin"
# Keil生成的bin文件路径
# ============================================================
# Boot单字节诊断码含义
SIG_DESC = {
    0xE0: "Boot连续5次解析包头失败，已放弃等待(请复位重试)",
    0xE1: "包头接收超时/不完整(检查TX接线和波特率)",
    0xE2: "包头CRC校验错误(传输有误或两边CRC算法不一致)",
    0xE3: "包头magic不匹配(字节错位/波特率不准)",
    0xE4: "固件长度非法(为0或超出分区大小)",
    0xE5: "固件分片接收超时(传输中断/RX接线松动)",
    0xE6: "写Flash失败(地址/半字对齐问题)",
    0xE7: "固件整体CRC校验失败(数据传输出错)",
    0xE8: "擦除App分区失败",
}
DEBUG_RAW_RX = True        # 调试开关：打印单片机发来的每一个原始字节
# 从串口读1个协议字节，自动跳过Boot发送时附带的\r\n换行符
def read_sig_byte(ser) -> int | None:
    while True:
        b = ser.read(1)
        if not b:
            return None            # 超时，无数据
        if DEBUG_RAW_RX:
            ch = chr(b[0]) if 0x20 <= b[0] <= 0x7E else "."
            print(f"    [RX] 0x{b[0]:02X} '{ch}'")
        if b[0] in (0x0D, 0x0A):   # 跳过 \r \n
            continue
        return b[0]
# 和Boot里面一模一样的CRC32函数
def crc32_custom(data:bytes) -> int:
    crc = 0xFFFFFFFF
    poly = 0xEDB88320
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return (~crc) & 0xFFFFFFFF
# 构造20字节包头，完全匹配你的boot代码
def build_header(fw_crc:int, fw_len:int, ver:int) -> bytes:
    head = bytearray(20)
    MAGIC_START = 0xAA55
    MAGIC_STOP  = 0x55AA
    # Magic Start 0xAA55 小端
    head[0] = MAGIC_START & 0xFF
    head[1] = (MAGIC_START >> 8) & 0xFF
    # 固件整体CRC
    head[2] = (fw_crc >> 0) & 0xFF
    head[3] = (fw_crc >> 8) & 0xFF
    head[4] = (fw_crc >>16) & 0xFF
    head[5] = (fw_crc >>24) & 0xFF
    # 固件长度
    head[6] = (fw_len >> 0) & 0xFF
    head[7] = (fw_len >> 8) & 0xFF
    head[8] = (fw_len >>16) & 0xFF
    head[9] = (fw_len >>24) & 0xFF
    # 版本
    head[10] = (ver >> 0) & 0xFF
    head[11] = (ver >> 8) & 0xFF
    head[12] = (ver >>16) & 0xFF
    head[13] = (ver >>24) & 0xFF
    # 计算包头CRC：head[2:14] 共12字节
    tmp = head[2:14]
    header_crc = crc32_custom(tmp)
    head[14] = (header_crc >> 0) & 0xFF
    head[15] = (header_crc >> 8) & 0xFF
    head[16] = (header_crc >>16) & 0xFF
    head[17] = (header_crc >>24) & 0xFF
    # Magic Stop 0x55AA 小端
    head[18] = MAGIC_STOP & 0xFF
    head[19] = (MAGIC_STOP >> 8) & 0xFF
    return bytes(head)
def main():
    # 读取bin文件
    with open(BIN_FILE_PATH,"rb") as f:
        bin_data = f.read()
    fw_len = len(bin_data)
    # Boot端按uint16_t半字写Flash，奇数长度补1字节0xFF（Flash擦除态本就是0xFF，无副作用）
    # 注意：补字节必须在算长度和CRC之前，让包头的len/crc覆盖补齐后的数据
    if fw_len % 2 != 0:
        bin_data += b"\xFF"
        print(f"固件原长 {fw_len} 为奇数，已补1字节0xFF，发送长度按 {len(bin_data)} 计")
    fw_len = len(bin_data)
    fw_crc = crc32_custom(bin_data)
    print(f"固件长度: {fw_len} 字节")
    print(f"固件CRC32: 0x{fw_crc:08X}")
    header = build_header(fw_crc, fw_len, VERSION)
    print("生成包头(hex):", header.hex(" "))
    # 检测目标串口是否存在（未插串口设备时直接退出）
    available_ports = [p.device for p in list_ports.comports()]
    if COM_PORT not in available_ports:
        print(f"无目标：未检测到串口 {COM_PORT}，当前可用串口: {available_ports or '无'}")
        return
    # 打开串口
    try:
        ser = Serial(COM_PORT, BAUDRATE, timeout=0.2)
    except SerialException as e:
        print(f"无目标：串口 {COM_PORT} 打开失败: {e}")
        return
    # pyserial打开时会默认拉高DTR/RTS，若模块将这两线接到STM32的NRST/BOOT0，
    # 会把单片机持续复位或拉进系统Bootloader，表现为"串口助手有数据、Python没数据"
    ser.dtr = False
    ser.rts = False
    print("串口打开成功，等待Boot请求...")
    print("提示：请给STM32上电/复位，使其进入Boot（30秒内）")
    # 请求-应答握手（不再盲发包头）：
    #   Boot发0x5A请求包头 -> 本程序回20字节包头
    #   Boot擦完Flash发0xA5 -> 本程序发固件
    #   任何0xEx都是Boot上报的具体失败原因；非协议字节通常是乱码(波特率/接线)
    ser.timeout = 1.0
    ser.reset_input_buffer()
    request_cnt = 0
    ready = False
    idle_secs = 0
    deadline = time.monotonic() + READY_TIMEOUT
    while time.monotonic() < deadline:
        sig = read_sig_byte(ser)
        if sig is None:
            idle_secs += 1
            if idle_secs % 5 == 0:
                print(f"...已等待{idle_secs}秒，串口无任何数据（若一直如此，检查串口助手是否已关闭/接线）")
            continue
        idle_secs = 0
        if sig == REQ_HEAD_BYTE:
            request_cnt += 1
            print(f"收到Boot包头请求(第{request_cnt}次)，发送包头")
            ser.write(header)
        elif sig == READY_BYTE:
            ready = True
            break
        elif 0xE0 <= sig <= 0xEF:
            print(f"[Boot报错] 0x{sig:02X} {SIG_DESC.get(sig, '未知错误码')}")
        else:
            print(f"[警告] 收到非协议字节 0x{sig:02X}（若持续刷随机字节，多为波特率不匹配/TX-RX接反）")
    if not ready:
        print(f"握手失败：{READY_TIMEOUT}秒内未收到Boot就绪字节0x{READY_BYTE:02X}")
        print("排查顺序：①Boot是否已重新编译烧录 ②PA9(TX)->模块RX、PA10(RX)->模块TX、共地 ③串口号/波特率")
        ser.close()
        return
    print("收到Boot就绪信号(0xA5)，开始发送固件分片")
    # 静默100ms：Boot发0xA5后会排空串口到50ms总线静默才开始收固件，
    # 这里延时(>50ms)确保首片固件不会被它当残留字节丢掉
    time.sleep(0.1)
    # 2.循环分片发送BIN；每发一片检查Boot是否中途上报错误
    send_ptr = 0
    total = len(bin_data)
    aborted = False
    while send_ptr < total:
        chunk = bin_data[send_ptr : send_ptr + CHUNK_SIZE]
        ser.write(chunk)
        send_ptr += len(chunk)
        print(f"\r已发送: {send_ptr}/{total}", end="")
        time.sleep(0.01) # 可根据需要调延时，防止单片接收溢出
        if ser.in_waiting:
            msg = ser.read(ser.in_waiting)
            if DEBUG_RAW_RX and msg:
                print(f"\n    [RX] {msg.hex(' ')}")
            for sig in msg:
                if sig in (0x0D, 0x0A):   # 跳过Boot信号附带的\r\n
                    continue
                if 0xE0 <= sig <= 0xEF:
                    print(f"\n[Boot报错] 0x{sig:02X} {SIG_DESC.get(sig, '未知错误码')}")
                    aborted = True
    if aborted:
        print("升级中止")
        ser.close()
        return
    print("\n固件全部发送完毕，等待Boot校验结果...")
    # 3.等最终结果：0x55=成功并跳转；0xEx=失败；超时=未收到结果码
    ser.timeout = RESULT_TIMEOUT
    result = read_sig_byte(ser)
    if result == SUCCESS_BYTE:
        print("[成功] Boot校验通过(0x55)，已跳转新App")
    elif result is not None and 0xE0 <= result <= 0xEF:
        print(f"[失败] 0x{result:02X} {SIG_DESC.get(result, '未知错误码')}")
    else:
        got = f"0x{result:02X}" if result is not None else "无"
        print(f"未收到Boot结果码(实际: {got})，请观察板子是否已跳转新App")
    ser.close()
if __name__ == "__main__":
    main()
