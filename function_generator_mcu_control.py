#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MSPM0G3507 「地猛星」函数发生器上位机 v3.0 — 二进制流式版
============================================================
- 通过串口以 11-byte 紧凑帧向 MCU 推送 DAC 采样点
- DDS 引擎在 PC 端计算，MCU 端纯播放
- 帧格式: [5A A5 CH 02 A1H A1L A2H A2L A3H A3L CK]
- CH 固定为 1（CH1），每帧 3 个 12-bit 样本
"""

import sys
import math
import time
import struct
import threading
import numpy as np
from typing import Optional

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QLineEdit, QSlider,
    QMessageBox, QStatusBar, QButtonGroup, QGridLayout,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QIntValidator
import pyqtgraph as pg

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("请安装 pyserial:  pip install pyserial")
    sys.exit(1)

# ============================================================================
#  常量
# ============================================================================
FREQ_MIN       = 1
FREQ_MAX       = 5000
AMP_MAX        = 4095
VREF           = 3.3
SAMPLE_RATE    = 100000      # 目标采样率
PHASE_MAX      = 0x100000000 # 2^32
SINE_TABLE_SZ  = 256

FRAME_H1       = 0x5A
FRAME_H2       = 0xA5
CMD_COMPACT    = 0x02
FRAME_LEN      = 11
SAMPLES_PER_FRAME = 3

WAVE_TYPES = {
    "无":     "none",
    "正弦波": "sine",
    "方波":   "square",
    "三角波": "triangle",
    "锯齿波": "sawtooth",
    "直流":   "dc",
    "噪声":   "noise",
}

WAVE_COLORS = {
    "正弦波": "#00E5FF", "方波": "#FFD700", "三角波": "#00FF88",
    "锯齿波": "#FF8C00", "直流": "#FF3333", "噪声": "#AA88FF",
    "无":     "#666666",
}

# ============================================================================
#  256 点 12-bit 正弦查表
# ============================================================================
_sine_lut = np.array([
    2047, 2097, 2147, 2198, 2248, 2298, 2347, 2397,
    2446, 2496, 2545, 2593, 2641, 2689, 2737, 2784,
    2831, 2877, 2922, 2968, 3012, 3056, 3100, 3142,
    3185, 3226, 3267, 3307, 3346, 3384, 3422, 3459,
    3495, 3530, 3564, 3597, 3630, 3661, 3692, 3721,
    3749, 3777, 3803, 3829, 3853, 3876, 3898, 3919,
    3939, 3957, 3975, 3991, 4006, 4020, 4033, 4045,
    4055, 4064, 4072, 4079, 4085, 4089, 4092, 4094,
    4095, 4094, 4092, 4089, 4085, 4079, 4072, 4064,
    4055, 4045, 4033, 4020, 4006, 3991, 3975, 3957,
    3939, 3919, 3898, 3876, 3853, 3829, 3803, 3777,
    3749, 3721, 3692, 3661, 3630, 3597, 3564, 3530,
    3495, 3459, 3422, 3384, 3346, 3307, 3267, 3226,
    3185, 3142, 3100, 3056, 3012, 2968, 2922, 2877,
    2831, 2784, 2737, 2689, 2641, 2593, 2545, 2496,
    2446, 2397, 2347, 2298, 2248, 2198, 2147, 2097,
    2047, 1997, 1947, 1896, 1846, 1796, 1747, 1697,
    1648, 1598, 1549, 1501, 1453, 1405, 1357, 1310,
    1263, 1217, 1172, 1126, 1082, 1038,  994,  952,
     909,  868,  827,  787,  748,  710,  672,  635,
     599,  564,  530,  497,  464,  433,  402,  373,
     345,  317,  291,  265,  241,  218,  196,  175,
     155,  137,  119,  103,   88,   74,   61,   49,
      39,   30,   22,   15,    9,    5,    2,    0,
       0,    0,    2,    5,    9,   15,   22,   30,
      39,   49,   61,   74,   88,  103,  119,  137,
     155,  175,  196,  218,  241,  265,  291,  317,
     345,  373,  402,  433,  464,  497,  530,  564,
     599,  635,  672,  710,  748,  787,  827,  868,
     909,  952,  994, 1038, 1082, 1126, 1172, 1217,
    1263, 1310, 1357, 1405, 1453, 1501, 1549, 1598,
    1648, 1697, 1747, 1796, 1846, 1896, 1947, 1997,
], dtype=np.uint16)


# ============================================================================
#  DDS 引擎 — 在 PC 端实时计算波形采样点
# ============================================================================
class DDSEngine:
    """32-bit 相位累加器 DDS，与 MCU 端数学等价"""

    def __init__(self):
        self._phase_acc = 0          # 32-bit 相位
        self._lfsr = 0x5EEDBEEF      # 噪声 LFSR

    def set_freq(self, freq_hz: float, sample_rate: float):
        """根据输出频率和有效采样率计算相位步进"""
        self._phase_step = int(PHASE_MAX * freq_hz / sample_rate)

    def reset_phase(self):
        self._phase_acc = 0

    def next_sample(self, wave_type: str, amplitude: int) -> int:
        """计算下一个 12-bit DAC 采样值"""
        self._phase_acc = (self._phase_acc + self._phase_step) & 0xFFFFFFFF
        ph = self._phase_acc
        half = 0x80000000

        if wave_type == "sine":
            idx = (ph >> 24) & 0xFF
            val = int(_sine_lut[idx])
            val = val * amplitude // 4095
        elif wave_type == "square":
            val = amplitude if ph < half else 0
        elif wave_type == "triangle":
            if ph < half:
                val = (ph * 2 * amplitude) >> 32
            else:
                val = ((0xFFFFFFFF - ph) * 2 * amplitude) >> 32
        elif wave_type == "sawtooth":
            val = (ph * amplitude) >> 32
        elif wave_type == "dc":
            val = amplitude
        elif wave_type == "noise":
            # 32-bit Galois LFSR
            lsb = self._lfsr & 1
            self._lfsr >>= 1
            if lsb:
                self._lfsr ^= 0x80200003
            val = (self._lfsr & 0xFFF) * amplitude // 4095
        else:  # none
            val = 0

        return max(0, min(4095, val))


# ============================================================================
#  串口流式发送线程 — 持续 DDS 计算 + 二进制帧推送
# ============================================================================
class StreamingThread(QThread):
    """后台线程：持续计算波形 → 打包紧凑帧 → 写入串口。
    不等待 MCU 响应，纯流式推送，UI 线程永不阻塞。"""

    # 统计信号
    stats_updated  = pyqtSignal(int, float)            # 总样本数, 实际速率
    echo_received  = pyqtSignal(int, int, int)         # 回传的 3 个 DAC 值
    status_updated = pyqtSignal(int, int, int, int)    # ok,bad,fill,lost
    error_occurred = pyqtSignal(str)

    def __init__(self, ser: serial.Serial, baudrate: int):
        super().__init__()
        self._ser = ser
        self._baudrate = baudrate
        self._running = False
        self._lock = threading.Lock()

        # 波形参数（由 UI 线程设置）
        self._wave_type = "sine"
        self._freq_hz = 1000.0
        self._amplitude = 4095
        self._active = False

        # DDS 引擎
        self._dds = DDSEngine()

    # ---- 线程安全参数更新 ----
    def set_params(self, wave_type: str, freq_hz: float, amplitude: int, active: bool):
        with self._lock:
            self._wave_type = wave_type
            self._freq_hz = freq_hz
            self._amplitude = amplitude
            self._active = active

    def stop_thread(self):
        self._running = False
        self.wait(2000)

    # ---- 响应解析（非阻塞） ----
    def _read_responses(self):
        """读取 MCU 回传的 ECHO 帧 & 状态行，不阻塞"""
        try:
            n = self._ser.in_waiting
            if n <= 0:
                return
            data = self._ser.read(min(n, 1024))
            i = 0
            ln = len(data)
            while i + FRAME_LEN <= ln:
                # 寻找帧头
                if data[i] != FRAME_H1 or data[i + 1] != FRAME_H2:
                    # 可能是文本状态行: "S:..."
                    if data[i] == ord('S') and data[i + 1] == ord(':'):
                        end = data.find(b'\n', i)
                        if end < 0:
                            break
                        line = data[i:end].decode('ascii', errors='ignore')
                        parts = line[2:].strip().split()
                        if len(parts) >= 4:
                            try:
                                ok   = int(parts[0], 16)
                                bad  = int(parts[1], 16)
                                fill = int(parts[2], 16)
                                lost = int(parts[3], 16)
                                self.status_updated.emit(ok, bad, fill, lost)
                            except ValueError:
                                pass
                        i = end + 1
                        continue
                    i += 1
                    continue

                # 读取完整 11 字节帧
                if i + FRAME_LEN > ln:
                    break
                ch  = data[i + 2]
                cmd = data[i + 3]
                ck_rx = data[i + 10]
                ck_calc = (sum(data[i:i + 10])) & 0xFF

                if ck_calc == ck_rx and ch == 1 and cmd == 0x03:
                    # ECHO 帧: 3 个 uint16 大端
                    s0 = (data[i + 4] << 8) | data[i + 5]
                    s1 = (data[i + 6] << 8) | data[i + 7]
                    s2 = (data[i + 8] << 8) | data[i + 9]
                    self.echo_received.emit(s0, s1, s2)
                    i += FRAME_LEN
                else:
                    i += 1  # 非 ECHO 帧，跳过

        except (serial.SerialException, OSError):
            pass

    # ---- 主循环 ----
    def run(self):
        # ── 计算自适应传输步长 ──
        # 每帧 11 字节，串口传输: baud/10 字节/秒
        max_fps = (self._baudrate / 10.0) / FRAME_LEN
        safe_fps = max_fps * 0.90
        transmit_step = max(1, int(math.ceil(SAMPLE_RATE / safe_fps)))
        effective_rate = SAMPLE_RATE / transmit_step

        # 更新 DDS 相位步进（以有效采样率为基准）
        self._dds._phase_step = int(PHASE_MAX * self._freq_hz / effective_rate)

        frame_interval = 1.0 / (effective_rate / SAMPLES_PER_FRAME)

        self._running = True
        epoch_start = time.perf_counter()
        t_last_stats = epoch_start
        next_slot = epoch_start
        frame_count = 0
        prev_active = False

        try:
            while self._running:
                # ── 快照当前参数 ──
                with self._lock:
                    wave = self._wave_type
                    freq = self._freq_hz
                    amp  = self._amplitude
                    active = self._active

                if not active:
                    # 未激活 → 休眠等待
                    self.msleep(50)
                    next_slot = time.perf_counter()
                    prev_active = False
                    continue

                # 激活状态切换时重置调度器 + 相位
                if not prev_active:
                    self._dds._phase_step = int(PHASE_MAX * freq / effective_rate)
                    self._dds.reset_phase()
                    next_slot = time.perf_counter()
                    prev_active = True

                # DDS 频率变化时更新步进
                self._dds._phase_step = int(PHASE_MAX * freq / effective_rate)

                # ── 精确等待到下一个时间槽 ──
                wait_s = next_slot - time.perf_counter()
                if wait_s > 0.002:
                    time.sleep(wait_s - 0.0015)
                while time.perf_counter() < next_slot:
                    pass

                # ── 生成一帧: 3 个 12-bit 样本 ──
                s0 = self._dds.next_sample(wave, amp)
                s1 = self._dds.next_sample(wave, amp)
                s2 = self._dds.next_sample(wave, amp)

                # 打包: [5A A5 01 02 A1H A1L A2H A2L A3H A3L CK]
                a1 = struct.pack('>H', s0 & 0xFFFF)
                a2 = struct.pack('>H', s1 & 0xFFFF)
                a3 = struct.pack('>H', s2 & 0xFFFF)
                ck = (FRAME_H1 + FRAME_H2 + 1 + CMD_COMPACT +
                      a1[0] + a1[1] + a2[0] + a2[1] + a3[0] + a3[1]) & 0xFF

                frame = bytearray([FRAME_H1, FRAME_H2, 1, CMD_COMPACT])
                frame.extend(a1)
                frame.extend(a2)
                frame.extend(a3)
                frame.append(ck)

                # ── 写入串口 ──
                try:
                    self._ser.write(frame)
                    frame_count += SAMPLES_PER_FRAME
                except (serial.SerialTimeoutException, serial.SerialException):
                    pass

                # ── 读取 MCU 回传（非阻塞） ──
                self._read_responses()

                # ── 推进时间槽 ──
                next_slot = max(next_slot + frame_interval, time.perf_counter())

                # ── 周期性统计 ──
                now = time.perf_counter()
                if now - t_last_stats >= 0.5:
                    elapsed = now - epoch_start
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    self.stats_updated.emit(frame_count, fps)
                    t_last_stats = now

        except Exception as e:
            self.error_occurred.emit(str(e))


# ============================================================================
#  波形预览引擎 — 仅用于本地显示
# ============================================================================
class WaveformPreviewEngine:
    @staticmethod
    def compute(wave_type: str, freq_hz: float, amplitude: int,
                t_ms: np.ndarray) -> np.ndarray:
        if wave_type in ("无", "none", ""):
            return np.zeros_like(t_ms)
        t_sec = t_ms / 1000.0
        ph = np.mod(2.0 * np.pi * freq_hz * t_sec, 2.0 * np.pi)
        amp_v = amplitude * VREF / AMP_MAX

        if wave_type == "正弦波" or wave_type == "sine":
            raw = 0.5 + 0.5 * np.sin(ph)
        elif wave_type == "方波" or wave_type == "square":
            raw = np.where(np.sin(ph) >= 0, 1.0, 0.0)
        elif wave_type == "三角波" or wave_type == "triangle":
            p = ph / (2.0 * np.pi)
            raw = np.where(p < 0.5, 2.0 * p, 2.0 * (1.0 - p))
        elif wave_type == "锯齿波" or wave_type == "sawtooth":
            raw = ph / (2.0 * np.pi)
        elif wave_type == "直流" or wave_type == "dc":
            raw = np.ones_like(ph)
        elif wave_type == "噪声" or wave_type == "noise":
            raw = np.random.rand(len(t_ms))
        else:
            raw = np.zeros_like(ph)
        return raw * amp_v


# ============================================================================
#  主 UI
# ============================================================================
class FunctionGeneratorControl(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MSPM0G3507 地猛星 函数发生器 v3.0 — Binary Streaming")
        self.setMinimumSize(1200, 700)

        self._serial: Optional[serial.Serial] = None
        self._streamer: Optional[StreamingThread] = None

        # 当前参数
        self._wave_type = "正弦波"
        self._freq_hz   = 1000
        self._amplitude = 4095
        self._output_on = False

        self._init_ui()

        self._scan_timer = QTimer(self)
        self._scan_timer.timeout.connect(self._scan_ports)
        self._scan_timer.start(2000)
        self._scan_ports()
        self._update_preview()

    # ============================ UI 构建 ============================

    def _init_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        root = QHBoxLayout(cw)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(12)
        root.addWidget(self._left_panel(), stretch=0)
        root.addWidget(self._right_panel(), stretch=1)

        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("系统就绪 — 请连接串口")

    def _left_panel(self) -> QWidget:
        p = QWidget()
        p.setFixedWidth(400)
        ly = QVBoxLayout(p); ly.setContentsMargins(0,0,0,0); ly.setSpacing(8)

        # 1. 串口
        grp = QGroupBox("1. 串口连接")
        g = QGridLayout(grp)
        g.addWidget(QLabel("端口:"), 0, 0)
        self._combo_port = QComboBox(); self._combo_port.setMinimumWidth(140)
        g.addWidget(self._combo_port, 0, 1)
        g.addWidget(QLabel("波特率:"), 1, 0)
        self._combo_baud = QComboBox()
        self._combo_baud.addItems(["115200","230400","460800","921600","1000000","2000000"])
        self._combo_baud.setCurrentText("2000000")
        g.addWidget(self._combo_baud, 1, 1)
        self._btn_conn = QPushButton("🔌 连接"); self._btn_conn.setMinimumHeight(36)
        self._btn_conn.clicked.connect(self._on_connect)
        self._style_conn_btn()
        g.addWidget(self._btn_conn, 2, 0, 1, 2)
        ly.addWidget(grp)

        # 2. 波形选择
        grp = QGroupBox("2. 波形选择")
        gv = QVBoxLayout(grp)
        self._wave_btns = {}
        # exclusive=False + 手动管理互斥，允许点击已选中按钮取消选择（→ "无"）
        self._wave_grp = QButtonGroup(self)
        self._wave_grp.setExclusive(False)
        idx = 0
        for row_names in [["正弦波","方波","三角波"], ["锯齿波","直流","噪声"]]:
            row = QHBoxLayout()
            for name in row_names:
                btn = QPushButton(name); btn.setCheckable(True); btn.setMinimumHeight(36)
                btn.setFont(QFont("Microsoft YaHei", 10))
                row.addWidget(btn)
                self._wave_btns[name] = btn
                self._wave_grp.addButton(btn, idx)  # 必须加入 Group 信号才生效
                idx += 1
            gv.addLayout(row)
        self._wave_btns["正弦波"].setChecked(True)
        self._style_wave("正弦波", True)
        self._wave_grp.buttonClicked.connect(self._on_wave)
        ly.addWidget(grp)

        # 3. 频率
        grp = QGroupBox("3. 频率 (Frequency)")
        gv = QVBoxLayout(grp)
        hr = QHBoxLayout()
        self._lbl_freq = QLabel("1000")
        self._lbl_freq.setFont(QFont("Consolas", 24, QFont.Bold))
        self._lbl_freq.setStyleSheet("color:#00FF88;background:#151520;padding:4px 12px;border-radius:4px;")
        hr.addWidget(self._lbl_freq); hr.addWidget(QLabel("Hz")); hr.addStretch()
        he = QHBoxLayout()
        he.addWidget(QLabel("数值:"))
        self._edit_freq = QLineEdit("1000")
        self._edit_freq.setValidator(QIntValidator(FREQ_MIN, FREQ_MAX))
        self._edit_freq.textEdited.connect(self._on_freq_edit)
        he.addWidget(self._edit_freq)
        self._slider_freq = QSlider(Qt.Horizontal)
        self._slider_freq.setRange(FREQ_MIN, FREQ_MAX)
        self._slider_freq.setValue(1000)
        self._slider_freq.valueChanged.connect(self._on_freq_slide)
        gv.addLayout(hr); gv.addLayout(he); gv.addWidget(self._slider_freq)
        ly.addWidget(grp)

        # 4. 幅值
        grp = QGroupBox("4. 幅值 (Amplitude)")
        gv = QVBoxLayout(grp)
        hr = QHBoxLayout()
        hr.addWidget(QLabel("DAC 码:"))
        self._edit_amp = QLineEdit("4095")
        self._edit_amp.setValidator(QIntValidator(0, AMP_MAX))
        self._edit_amp.textEdited.connect(self._on_amp_edit)
        hr.addWidget(self._edit_amp)
        self._lbl_amp = QLabel("(3.30 Vpp)")
        self._lbl_amp.setStyleSheet("color:#FFD700;font-weight:bold;")
        hr.addWidget(self._lbl_amp)
        self._slider_amp = QSlider(Qt.Horizontal)
        self._slider_amp.setRange(0, AMP_MAX); self._slider_amp.setValue(4095)
        self._slider_amp.valueChanged.connect(self._on_amp_slide)
        gv.addLayout(hr); gv.addWidget(self._slider_amp)
        ly.addWidget(grp)

        # 5. 输出控制
        grp = QGroupBox("5. 输出控制")
        gv = QHBoxLayout(grp)
        self._btn_out = QPushButton("▶ 启动输出 (START)")
        self._btn_out.setCheckable(True); self._btn_out.setMinimumHeight(48)
        self._btn_out.clicked.connect(self._on_output)
        self._style_output_btn()
        gv.addWidget(self._btn_out)
        ly.addWidget(grp)

        # 6. 状态
        grp = QGroupBox("6. 状态")
        gv = QVBoxLayout(grp)
        self._lbl_status = QLabel("⚪ 未连接")
        self._lbl_status.setFont(QFont("Consolas", 10))
        self._lbl_status.setStyleSheet("color:#888;padding:4px;")
        gv.addWidget(self._lbl_status)
        self._lbl_stats = QLabel("")
        self._lbl_stats.setFont(QFont("Consolas", 9))
        self._lbl_stats.setStyleSheet("color:#555;")
        gv.addWidget(self._lbl_stats)
        ly.addWidget(grp)
        ly.addStretch()
        return p

    def _right_panel(self) -> QWidget:
        p = QWidget()
        ly = QVBoxLayout(p); ly.setContentsMargins(0,0,0,0)
        hdr = QHBoxLayout()
        hdr.addWidget(QLabel("波形预览 (Preview)"))
        hdr.addStretch()
        self._lbl_info = QLabel("f=1000Hz  A=4095  Vpp=3.30V")
        self._lbl_info.setStyleSheet("color:#888;font-family:Consolas;")
        hdr.addWidget(self._lbl_info)
        ly.addLayout(hdr)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("#0A0A0F")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left", "Voltage", units="V")
        self._plot.setLabel("bottom", "Time", units="ms")
        self._plot.enableAutoRange(x=False, y=False)
        self._plot.setXRange(0, 10, padding=0)
        self._plot.setYRange(-0.2, VREF+0.3, padding=0)
        self._curve = self._plot.plot(pen=pg.mkPen("#00E5FF", width=2.5))
        self._ref_line = pg.InfiniteLine(angle=0, pos=VREF/2,
            pen=pg.mkPen(color="#FFFFFF22", style=Qt.DashLine, width=1))
        self._plot.addItem(self._ref_line)
        ly.addWidget(self._plot)
        return p

    # ============================ UI 样式 ============================

    def _style_wave(self, name: str, on: bool):
        btn = self._wave_btns[name]
        c = WAVE_COLORS.get(name, "#0078D4")
        if on:
            btn.setStyleSheet(f"QPushButton{{background-color:{c};color:#000;"
                "font-weight:bold;border:2px solid white;border-radius:6px;"
                "min-height:36px;font-size:13px;}}")
        else:
            btn.setStyleSheet("QPushButton{background-color:#2A2A35;color:#888;"
                "border:1px solid #444;border-radius:6px;min-height:36px;font-size:13px;}"
                "QPushButton:hover{background-color:#353545;color:#CCC;}")

    def _style_conn_btn(self):
        if self._serial and self._serial.is_open:
            self._btn_conn.setText("🔌 断开")
            self._btn_conn.setStyleSheet("QPushButton{background-color:#D13438;color:white;"
                "font-weight:bold;border-radius:4px;font-size:13px;min-height:36px;}"
                "QPushButton:hover{background-color:#A4262C;}")
        else:
            self._btn_conn.setText("🔌 连接")
            self._btn_conn.setStyleSheet("QPushButton{background-color:#0078D4;color:white;"
                "font-weight:bold;border-radius:4px;font-size:13px;min-height:36px;}"
                "QPushButton:hover{background-color:#106EBE;}")

    def _style_output_btn(self):
        if self._output_on:
            self._btn_out.setText("⏸ 停止输出 (STOP)")
            self._btn_out.setStyleSheet("QPushButton{background-color:#FF6600;color:white;"
                "font-weight:bold;border-radius:6px;font-size:16px;min-height:48px;"
                "border:2px solid #FF9944;}QPushButton:hover{background-color:#CC5500;}")
        else:
            self._btn_out.setText("▶ 启动输出 (START)")
            self._btn_out.setStyleSheet("QPushButton{background-color:#00AA44;color:white;"
                "font-weight:bold;border-radius:6px;font-size:16px;min-height:48px;"
                "border:2px solid #44CC66;}QPushButton:hover{background-color:#008833;}")

    # ============================ 参数更新 → 流式线程 ============================

    def _push_params(self):
        """将所有参数推送到后台流式线程（不阻塞 UI）"""
        if self._streamer and self._streamer.isRunning():
            wave_en = WAVE_TYPES.get(self._wave_type, "none")
            self._streamer.set_params(wave_en, self._freq_hz, self._amplitude, self._output_on)

    def _on_wave(self, btn: QPushButton):
        name = btn.text()
        if btn.isChecked():
            for b in self._wave_btns.values():
                if b is not btn:
                    b.blockSignals(True); b.setChecked(False); b.blockSignals(False)
                    self._style_wave(b.text(), False)
            self._style_wave(name, True)
            self._wave_type = name
        else:
            self._style_wave(name, False)
            self._wave_type = "无"
        self._push_params()
        self._update_preview()

    def _on_freq_slide(self, val: int):
        self._freq_hz = val
        self._edit_freq.setText(str(val))
        self._lbl_freq.setText(f"{val:,}".replace(",", " "))
        self._push_params()
        self._update_preview()

    def _on_freq_edit(self, text: str):
        try:
            v = int(text)
            if FREQ_MIN <= v <= FREQ_MAX:
                self._freq_hz = v
                self._slider_freq.blockSignals(True); self._slider_freq.setValue(v); self._slider_freq.blockSignals(False)
                self._lbl_freq.setText(f"{v:,}".replace(",", " "))
                self._push_params()
                self._update_preview()
        except ValueError: pass

    def _on_amp_slide(self, val: int):
        self._amplitude = val
        self._edit_amp.setText(str(val))
        self._lbl_amp.setText(f"({val*VREF/AMP_MAX:.2f} Vpp)")
        self._push_params()
        self._update_preview()

    def _on_amp_edit(self, text: str):
        try:
            v = int(text)
            if 0 <= v <= AMP_MAX:
                self._amplitude = v
                self._slider_amp.blockSignals(True); self._slider_amp.setValue(v); self._slider_amp.blockSignals(False)
                self._lbl_amp.setText(f"({v*VREF/AMP_MAX:.2f} Vpp)")
                self._push_params()
                self._update_preview()
        except ValueError: pass

    def _on_output(self, checked: bool):
        self._output_on = checked
        self._style_output_btn()
        self._push_params()

    # ============================ 串口连接管理 ============================

    def _on_connect(self):
        if self._serial and self._serial.is_open:
            # 断开
            if self._streamer:
                self._streamer.stop_thread()
                self._streamer = None
            self._serial.close()
            self._serial = None
            self._combo_port.setEnabled(True)
            self._combo_baud.setEnabled(True)
            self._style_conn_btn()
            self._lbl_status.setText("⚪ 已断开")
            self._status_bar.showMessage("串口已断开")
        else:
            # 连接
            port = self._combo_port.currentText()
            baud = int(self._combo_baud.currentText())
            if not port or port.startswith("("):
                QMessageBox.warning(self, "错误", "请选择有效串口。"); return
            try:
                self._serial = serial.Serial(port=port, baudrate=baud,
                    timeout=0.01, write_timeout=0.01)
            except Exception as e:
                QMessageBox.critical(self, "串口错误", str(e)); return

            self._combo_port.setEnabled(False)
            self._combo_baud.setEnabled(False)
            self._style_conn_btn()
            self._lbl_status.setText("🟢 已连接 — 流式推送中")

            # 启动流式线程
            self._streamer = StreamingThread(self._serial, baud)
            self._streamer.stats_updated.connect(self._on_stats)
            self._streamer.echo_received.connect(self._on_echo)
            self._streamer.status_updated.connect(self._on_status)
            self._streamer.error_occurred.connect(self._on_stream_err)
            self._streamer.set_params(
                WAVE_TYPES.get(self._wave_type, "none"),
                self._freq_hz, self._amplitude, self._output_on)
            self._streamer.start()

            self._status_bar.showMessage(f"已连接 {port} @ {baud} bps — 流式推送")

    def _on_stats(self, total: int, fps: float):
        self._lbl_stats.setText(f"已推送: {total} 样本 | 速率: {fps:.0f} sps")

    def _on_echo(self, s0: int, s1: int, s2: int):
        """MCU 回传刚收到的 3 个 DAC 值"""
        self._lbl_stats.setText(
            f"已推送 | MCU回传: [{s0},{s1},{s2}] "
            f"({s0*VREF/AMP_MAX:.2f}V, {s1*VREF/AMP_MAX:.2f}V, {s2*VREF/AMP_MAX:.2f}V)")

    def _on_status(self, ok: int, bad: int, fill: int, lost: int):
        """MCU 周期状态上报"""
        self._status_bar.showMessage(
            f"MCU状态 — 接收帧:{ok} 坏帧:{bad} 缓冲:{fill} 丢帧:{lost}")

    def _on_stream_err(self, msg: str):
        self._lbl_status.setText(f"🔴 流错误: {msg}")

    def _scan_ports(self):
        if self._serial and self._serial.is_open: return
        cur = self._combo_port.currentText()
        ports = serial.tools.list_ports.comports()
        names = sorted(set(p.device for p in ports))
        self._combo_port.blockSignals(True); self._combo_port.clear()
        if names:
            self._combo_port.addItems(names)
            if cur in names: self._combo_port.setCurrentText(cur)
        else:
            self._combo_port.addItem("(未检测到串口)")
        self._combo_port.blockSignals(False)

    # ============================ 本地预览 ============================

    def _update_preview(self):
        t = np.linspace(0, 10.0, 1200)
        v = WaveformPreviewEngine.compute(self._wave_type, self._freq_hz, self._amplitude, t)
        c = WAVE_COLORS.get(self._wave_type, "#00E5FF")
        self._curve.setPen(pg.mkPen(c, width=2.5))
        self._curve.setData(t, v)
        vpp = self._amplitude * VREF / AMP_MAX
        self._lbl_info.setText(f"f={self._freq_hz}Hz  |  A={self._amplitude}/4095  |  Vpp={vpp:.2f}V  |  {self._wave_type}")

    # ============================ 关闭 ============================

    def closeEvent(self, event):
        self._scan_timer.stop()
        if self._streamer:
            self._streamer.stop_thread()
        if self._serial and self._serial.is_open:
            try: self._serial.close()
            except: pass
        event.accept()


# ============================================================================
#  入口
# ============================================================================
def main():
    try:
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    except AttributeError: pass

    app = QApplication(sys.argv)
    app.setFont(QFont("Microsoft YaHei", 9))
    app.setStyleSheet("""
        QMainWindow{background-color:#101014;}
        QGroupBox{font-weight:bold;border:1px solid #2D2D3D;border-radius:6px;
            margin-top:10px;padding-top:14px;color:#A5A5B5;}
        QGroupBox::title{subcontrol-origin:margin;left:12px;padding:0 3px;color:#008BE3;}
        QLabel{color:#B0B0BD;}
        QComboBox{background:#1A1A26;color:#DDD;border:1px solid #3A3A4A;
            padding:3px 6px;border-radius:4px;}
        QComboBox QAbstractItemView{background:#1A1A26;color:#DDD;selection-background-color:#0078D4;}
        QLineEdit{background:#1A1A26;color:#FFF;border:1px solid #3A3A4A;
            padding:4px 6px;border-radius:4px;font-family:Consolas;}
        QSlider::groove:horizontal{height:5px;background:#252535;border-radius:2px;}
        QSlider::handle:horizontal{background:#0078D4;border:1px solid #005A9E;
            width:16px;margin:-5px 0;border-radius:8px;}
        QPushButton{background-color:#2A2A35;color:#AAA;border:1px solid #444;
            border-radius:4px;min-height:30px;}
        QPushButton:hover{background-color:#353545;color:#FFF;}
        QPushButton:checked{background-color:#0078D4;color:white;font-weight:bold;}
        QStatusBar{color:#777;background:#0A0A0F;border-top:1px solid #222;}
    """)
    win = FunctionGeneratorControl()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
