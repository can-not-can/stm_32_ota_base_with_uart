import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext

import serial
from serial import Serial, SerialException
from serial.tools import list_ports

# 复用Send.py中已验证的协议逻辑（导入不会执行main，有__main__保护）
from Send import (crc32_custom, build_header, SIG_DESC,
                  REQ_HEAD_BYTE, READY_BYTE, SUCCESS_BYTE,
                  CHUNK_SIZE, READY_TIMEOUT, RESULT_TIMEOUT)

PARITY_OPTS = {
    "无(None)": serial.PARITY_NONE,
    "偶校验(Even)": serial.PARITY_EVEN,
    "奇校验(Odd)": serial.PARITY_ODD,
    "Mark": serial.PARITY_MARK,
    "Space": serial.PARITY_SPACE,
}
STOPBITS_OPTS = {"1": serial.STOPBITS_ONE,
                 "1.5": serial.STOPBITS_ONE_POINT_FIVE,
                 "2": serial.STOPBITS_TWO}
BYTESIZE_OPTS = {"8": serial.EIGHTBITS, "7": serial.SEVENBITS}
BAUDRATES = ["9600", "19200", "38400", "57600", "115200",
             "230400", "460800", "921600"]


def ota_worker(cfg, log_q, stop_ev):
    """串口升级工作线程：与Send.py main()相同的请求-应答流程"""
    def log(msg):
        log_q.put(msg)

    ser = None
    try:
        with open(cfg["bin_path"], "rb") as f:
            bin_data = f.read()
        fw_len = len(bin_data)
        if fw_len % 2 != 0:  # Boot按半字写Flash，奇数补0xFF（在算len/crc之前补）
            bin_data += b"\xFF"
            log(f"固件原长 {fw_len} 为奇数，已补1字节0xFF")
        fw_len = len(bin_data)
        fw_crc = crc32_custom(bin_data)
        header = build_header(fw_crc, fw_len, cfg["version"])
        log(f"固件长度: {fw_len} 字节")
        log(f"固件CRC32: 0x{fw_crc:08X}")
        log(f"生成包头(hex): {header.hex(' ')}")

        ser = Serial(cfg["port"], cfg["baudrate"],
                     bytesize=cfg["bytesize"], parity=cfg["parity"],
                     stopbits=cfg["stopbits"], timeout=1.0)
        ser.dtr = False   # 防止DTR/RTS经自动下载电路复位MCU
        ser.rts = False
        ser.reset_input_buffer()
        log(f"串口 {cfg['port']} 打开成功，等待Boot请求（{READY_TIMEOUT}秒）...")
        log("提示：请给STM32上电/复位，使其进入Boot")

        # 1.握手：等0x5A回包头，等0xA5发固件
        request_cnt = 0
        ready = False
        deadline = time.monotonic() + READY_TIMEOUT
        while time.monotonic() < deadline and not stop_ev.is_set():
            sig = read_sig_byte(ser, log)
            if sig is None:
                continue
            if sig == REQ_HEAD_BYTE:
                request_cnt += 1
                log(f"收到Boot包头请求(第{request_cnt}次)，发送包头")
                ser.write(header)
            elif sig == READY_BYTE:
                ready = True
                break
            elif 0xE0 <= sig <= 0xEF:
                log(f"[Boot报错] 0x{sig:02X} {SIG_DESC.get(sig, '未知错误码')}")
            else:
                log(f"[警告] 收到非协议字节 0x{sig:02X}（持续乱码多为波特率/接线问题）")
        if not ready:
            if not stop_ev.is_set():
                log(f"握手失败：{READY_TIMEOUT}秒内未收到Boot就绪字节0x{READY_BYTE:02X}")
                log("排查：①Boot已重新烧录 ②PA9(TX)->模块RX、PA10(RX)->模块TX、共地 ③串口号/波特率")
            return
        log("收到Boot就绪信号(0xA5)，开始发送固件分片")
        time.sleep(0.1)  # 等Boot排空残留字节(它排空到50ms静默)

        # 2.分片发送
        send_ptr, total, aborted = 0, len(bin_data), False
        while send_ptr < total and not stop_ev.is_set():
            chunk = bin_data[send_ptr: send_ptr + CHUNK_SIZE]
            ser.write(chunk)
            send_ptr += len(chunk)
            log_q.put(("PROGRESS", send_ptr, total))
            time.sleep(0.01)
            if ser.in_waiting:
                msg = ser.read(ser.in_waiting)
                log(f"    [RX] {msg.hex(' ')}")
                for s in msg:
                    if 0xE0 <= s <= 0xEF:
                        log(f"[Boot报错] 0x{s:02X} {SIG_DESC.get(s, '未知错误码')}")
                        aborted = True
        if aborted or stop_ev.is_set():
            log("升级中止")
            return
        log("固件全部发送完毕，等待Boot校验结果...")

        # 3.最终结果
        ser.timeout = RESULT_TIMEOUT
        result = read_sig_byte(ser, log)
        if result == SUCCESS_BYTE:
            log("[成功] Boot校验通过(0x55)，已跳转新App")
        elif result is not None and 0xE0 <= result <= 0xEF:
            log(f"[失败] 0x{result:02X} {SIG_DESC.get(result, '未知错误码')}")
        else:
            got = f"0x{result:02X}" if result is not None else "无"
            log(f"未收到Boot结果码(实际: {got})，请观察板子是否已跳转新App")
    except FileNotFoundError:
        log("错误：bin文件不存在，请重新选择")
    except SerialException as e:
        log(f"串口错误: {e}")
    except Exception as e:
        log(f"异常: {type(e).__name__}: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        log_q.put(("DONE",))


def read_sig_byte(ser, log):
    """读1个协议字节，自动跳过Boot信号附带的\r\n"""
    while True:
        b = ser.read(1)
        if not b:
            return None
        ch = chr(b[0]) if 0x20 <= b[0] <= 0x7E else "."
        log(f"    [RX] 0x{b[0]:02X} '{ch}'")
        if b[0] in (0x0D, 0x0A):
            continue
        return b[0]


class OtaApp:
    def __init__(self, root):
        self.root = root
        root.title("STM32 OTA 串口升级工具")
        self.log_q = queue.Queue()
        self.stop_ev = threading.Event()
        self.worker = None

        frm = ttk.Frame(root, padding=8)
        frm.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        # 串口区
        ttk.Label(frm, text="串口号").grid(row=0, column=0, sticky="w")
        self.cb_port = ttk.Combobox(frm, width=10, state="readonly")
        self.cb_port.grid(row=0, column=1, sticky="w")
        ttk.Button(frm, text="刷新", width=6,
                   command=self.refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(frm, text="波特率").grid(row=0, column=3, sticky="w", padx=(12, 0))
        self.cb_baud = ttk.Combobox(frm, width=9, values=BAUDRATES, state="readonly")
        self.cb_baud.set("115200")
        self.cb_baud.grid(row=0, column=4, sticky="w")

        ttk.Label(frm, text="数据位").grid(row=1, column=0, sticky="w")
        self.cb_bits = ttk.Combobox(frm, width=10, state="readonly",
                                    values=list(BYTESIZE_OPTS.keys()))
        self.cb_bits.set("8")
        self.cb_bits.grid(row=1, column=1, sticky="w")

        ttk.Label(frm, text="校验位").grid(row=1, column=3, sticky="w", padx=(12, 0))
        self.cb_parity = ttk.Combobox(frm, width=12, state="readonly",
                                      values=list(PARITY_OPTS.keys()))
        self.cb_parity.set("无(None)")
        self.cb_parity.grid(row=1, column=4, sticky="w")

        ttk.Label(frm, text="停止位").grid(row=1, column=5, sticky="w", padx=(12, 0))
        self.cb_stop = ttk.Combobox(frm, width=6, state="readonly",
                                    values=list(STOPBITS_OPTS.keys()))
        self.cb_stop.set("1")
        self.cb_stop.grid(row=1, column=6, sticky="w")

        # bin文件
        ttk.Label(frm, text="bin文件").grid(row=2, column=0, sticky="w")
        self.ent_bin = ttk.Entry(frm, width=55)
        self.ent_bin.grid(row=2, column=1, columnspan=5, sticky="we", pady=2)
        ttk.Button(frm, text="浏览...", command=self.browse_bin).grid(row=2, column=6)

        # 版本号
        ttk.Label(frm, text="版本号").grid(row=3, column=0, sticky="w")
        self.ent_ver = ttk.Entry(frm, width=10)
        self.ent_ver.insert(0, "1")
        self.ent_ver.grid(row=3, column=1, sticky="w")

        # 开始/停止按钮
        self.btn = ttk.Button(frm, text="开始升级", width=16, command=self.toggle)
        self.btn.grid(row=3, column=4, columnspan=2, pady=4)

        # 日志区
        self.txt = scrolledtext.ScrolledText(frm, width=85, height=22,
                                             font=("Consolas", 9))
        self.txt.grid(row=4, column=0, columnspan=7, sticky="nsew", pady=4)
        frm.rowconfigure(4, weight=1)
        frm.columnconfigure(1, weight=1)

        # 默认值取自Send.py配置，省去重复填写
        try:
            from Send import BIN_FILE_PATH, COM_PORT
            self.ent_bin.insert(0, BIN_FILE_PATH)
            self.refresh_ports()
            if COM_PORT in self.cb_port["values"]:
                self.cb_port.set(COM_PORT)
        except Exception:
            self.refresh_ports()

        self.root.after(100, self.poll_log)

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.cb_port["values"] = ports
        if ports and not self.cb_port.get():
            self.cb_port.set(ports[0])

    def browse_bin(self):
        path = filedialog.askopenfilename(
            title="选择固件bin文件", filetypes=[("BIN文件", "*.bin"), ("所有文件", "*.*")])
        if path:
            self.ent_bin.delete(0, tk.END)
            self.ent_bin.insert(0, path)

    def toggle(self):
        if self.worker and self.worker.is_alive():
            self.stop_ev.set()
            self.btn.config(text="停止中...", state="disabled")
            return
        port = self.cb_port.get()
        if not port:
            self.log("错误：未选择串口")
            return
        try:
            version = int(self.ent_ver.get(), 0) & 0xFFFFFFFF
        except ValueError:
            self.log("错误：版本号必须是整数")
            return
        cfg = {
            "port": port,
            "baudrate": int(self.cb_baud.get()),
            "bytesize": BYTESIZE_OPTS[self.cb_bits.get()],
            "parity": PARITY_OPTS[self.cb_parity.get()],
            "stopbits": STOPBITS_OPTS[self.cb_stop.get()],
            "bin_path": self.ent_bin.get().strip(),
            "version": version,
        }
        self.stop_ev.clear()
        self.btn.config(text="停止")
        self.worker = threading.Thread(
            target=ota_worker, args=(cfg, self.log_q, self.stop_ev), daemon=True)
        self.worker.start()

    def log(self, msg):
        self.txt.insert(tk.END, msg + "\n")
        self.txt.see(tk.END)

    def poll_log(self):
        try:
            while True:
                item = self.log_q.get_nowait()
                if isinstance(item, tuple):
                    if item[0] == "PROGRESS":
                        _, sent, total = item
                        self.txt.delete("end-2l", "end-1l")
                        self.log(f"已发送: {sent}/{total}")
                    elif item[0] == "DONE":
                        self.btn.config(text="开始升级", state="normal")
                else:
                    self.log(item)
        except queue.Empty:
            pass
        self.root.after(100, self.poll_log)


if __name__ == "__main__":
    root = tk.Tk()
    OtaApp(root)
    root.mainloop()
