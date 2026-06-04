#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Digital Storage Oscilloscope (DSO) v6.8 - Multi-Channel Edition
==================================================================
- Matches 4-Channel Function Generator protocol (11-byte frame)
- High-speed serial parsing supporting up to 2,000,000 baud
- Multi-channel time-aligned software triggering (AUTO / NORMAL)
- Color scheme: CH1(Yellow), CH2(Cyan), CH3(Magenta), CH4(Green)
"""

import sys
import numpy as np
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QStatusBar, QMessageBox,
    QButtonGroup, QSlider, QCheckBox, QGridLayout
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont

import pyqtgraph as pg

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Please install pyserial: pip install pyserial")
    sys.exit(1)

# ============================================================================
#  Constants
# ============================================================================

DEFAULT_BAUD = 921600          # Resolved bandwidth bottleneck (e.g., 921600, 2000000)
RX_HEAD1     = 0x5A            # v6.8 Frame Header 1
RX_HEAD2     = 0xA5            # v6.8 Frame Header 2
RX_FRAME_LEN = 11              # [5A, A5, ch_id, CMD, T3, T2, T1, T0, AH, AL, CK]

VREF         = 3.3
ADC_MAX      = 4095

H_DIVS       = 10              
V_DIVS       = 8               

TIME_DIV_OPTIONS  = [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1]
VOLTS_DIV_OPTIONS = [0.1, 0.2, 0.5, 1.0, 2.0]

DEFAULT_TIME_DIV  = 0.002
DEFAULT_VOLTS_DIV = 0.5

REFRESH_MS   = 30
DATA_BUF_MAX = 100000          # Buffer limit per channel
GAP_THRESHOLD_MS = 0.3         # 相邻点时间间隔 > 0.3ms 视为批次空洞，断线绘制

# UI Style Palette
CLR_BG       = "#121218"
CLR_TRIGGER  = "#FF3333"

# Standard Hardware Channel Colors (Yellow, Cyan, Magenta, Green)
CLR_CH = {
    1: "#FFD700",  # CH1: Yellow
    2: "#00E5FF",  # CH2: Cyan
    3: "#FF00FF",  # CH3: Magenta
    4: "#00FF88"   # CH4: Green
}


# ============================================================================
#  Serial Receive Thread (v6.8 11-Byte FSM Parser)
# ============================================================================

class MultiChannelSerialRxThread(QThread):
    # 信号发射格式：(通道号, ADC数据数组, 时间戳数组)
    samples_ready  = pyqtSignal(int, np.ndarray, np.ndarray)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: str, baud: int):
        super().__init__()
        self._port     = port
        self._baud     = baud
        self._serial   = None
        self._running  = False

    def run(self):
        try:
            # 开启高效串口驱动，适当调大输入缓冲区
            self._serial = serial.Serial(port=self._port, baudrate=self._baud, timeout=0.02)
            self._serial.set_buffer_size(rx_size=1024 * 1024) # 分配1MB缓冲区防止底层溢出
        except Exception as e:
            self.error_occurred.emit(f"无法打开串口 {self._port}: {e}")
            return

        self._running = True

        # 紧凑帧合成时间戳 (CMD=0x02) — 纳秒精度累加器，消除整数截断漂移
        import math
        SAMPLE_RATE = 100000
        max_fps = self._baud / 10.0 / 11.0
        safe_fps = max_fps * 0.90
        tx_step = math.ceil(SAMPLE_RATE / safe_fps)  # 假设 1 活跃通道
        if tx_step < 1: tx_step = 1
        actual_fps = SAMPLE_RATE / tx_step
        effective_rate = actual_fps * 3  # 3 样本/紧凑帧
        self._dt_ns = int(1_000_000_000 / effective_rate)  # 纳秒/样本
        self._synth_t_ns = {ch: 0 for ch in range(1, 5)}  # 纳秒累加器

        # 采用增量式高效 bytearray，避免频繁内存重分配
        raw_buf = bytearray()
        
        # 4通道独立批处理双缓冲 (提升 PyqtGraph 渲染效率，避免单点单发)
        BATCH_SIZE = 120
        adc_batches = {ch: [] for ch in range(1, 5)}
        tms_batches = {ch: [] for ch in range(1, 5)}

        while self._running:
            try:
                ser = self._serial
                if ser is None or not ser.is_open:
                    self.msleep(10)
                    continue

                n = ser.in_waiting
                if n == 0:
                    self.usleep(200) # 微秒级休眠，平衡低延迟与低CPU占用
                    continue

                # 读取原始字节流并追加
                chunk = ser.read(min(n, 16384))
                raw_buf.extend(chunk)

                # ========================================================
                #  高效滑动窗口状态机 (FSM Pointer 模式)
                #  支持两种帧格式：CMD=0x01 (单样本+时间戳) / CMD=0x02 (3样本紧凑帧)
                # ========================================================
                ptr = 0
                buf_len = len(raw_buf)

                # 只要剩余未解析字节大于等于一帧长度，就持续循环
                while (buf_len - ptr) >= RX_FRAME_LEN:
                    # 状态 0 & 状态 1：双字节帧头锁定 (0x5A, 0xA5)
                    if raw_buf[ptr] != RX_HEAD1 or raw_buf[ptr + 1] != RX_HEAD2:
                        ptr += 1  # 状态不匹配，窗口向后滑动 1 字节（退回状态0）
                        continue

                    # 提取潜在的一帧数据 (防止指针越界，前面已做边界判断)
                    # 对应协议：Byte0=5A, Byte1=A5, Byte2=ch_id, Byte3=CMD
                    ch_id = raw_buf[ptr + 2]
                    cmd   = raw_buf[ptr + 3]

                    # 状态 2 & 状态 3：验证通道有效性与命令字
                    if not (1 <= ch_id <= 4) or (cmd != 0x01 and cmd != 0x02):
                        ptr += 1  # 协议不合规，退回状态 0，滑动 1 字节寻找新帧头
                        continue

                    # 状态 6：校验和末尾验证 (累加前10个字节)
                    ck_rx   = raw_buf[ptr + 10]
                    ck_calc = sum(raw_buf[ptr : ptr + 10]) & 0xFF

                    if ck_calc != ck_rx:
                        # 校验失败，说明该段数据受硬件串扰。
                        # 安全处理：仅视当前 0x5A 失效，向前滑动 1 字节重新检索
                        ptr += 1
                        continue

                    if cmd == 0x02:
                        # ====================================================
                        #  紧凑帧格式 (3 样本/帧，零时间戳)
                        #  [5A, A5, ch_id, 02, A1H, A1L, A2H, A2L, A3H, A3L, CK]
                        #  接收端用合成时间戳均匀铺开
                        # ====================================================
                        a1 = (raw_buf[ptr + 4] << 8) | raw_buf[ptr + 5]
                        a2 = (raw_buf[ptr + 6] << 8) | raw_buf[ptr + 7]
                        a3 = (raw_buf[ptr + 8] << 8) | raw_buf[ptr + 9]

                        for adc_raw in (a1, a2, a3):
                            adc_val = max(0, min(ADC_MAX, adc_raw))
                            # 纳秒精度合成时间戳，消除整数截断漂移
                            synth_ns = self._synth_t_ns.get(ch_id, 0)
                            self._synth_t_ns[ch_id] = synth_ns + self._dt_ns
                            tms_batches[ch_id].append(synth_ns // 1000)  # ns → us
                            adc_batches[ch_id].append(adc_val)

                            # 触发分批投递机制
                            if len(adc_batches[ch_id]) >= BATCH_SIZE:
                                self.samples_ready.emit(
                                    ch_id,
                                    np.array(adc_batches[ch_id], dtype=np.uint16),
                                    np.array(tms_batches[ch_id], dtype=np.uint32)
                                )
                                adc_batches[ch_id].clear()
                                tms_batches[ch_id].clear()
                    else:
                        # ====================================================
                        #  标准帧格式 (1 样本/帧，带 4 字节时间戳)
                        #  [5A, A5, ch_id, 01, T3, T2, T1, T0, AH, AL, CK]
                        # ====================================================
                        # 1. 4字节时间戳拼装，依然保持完美的无符号大端序
                        t_us = ((raw_buf[ptr + 4] << 24) |
                                (raw_buf[ptr + 5] << 16) |
                                (raw_buf[ptr + 6] <<  8) |
                                 raw_buf[ptr + 7]) & 0xFFFFFFFF

                        # 2. ADC 解析 + 防下溢
                        adc_raw = (raw_buf[ptr + 8] << 8) | raw_buf[ptr + 9]
                        if adc_raw >= 32768:
                            adc_raw -= 65536
                        adc_val = max(0, min(ADC_MAX, adc_raw))

                        # 数据压入对应通道的分流缓冲区
                        adc_batches[ch_id].append(adc_val)
                        tms_batches[ch_id].append(t_us)

                        # 触发分批投递机制
                        if len(adc_batches[ch_id]) >= BATCH_SIZE:
                            self.samples_ready.emit(
                                ch_id,
                                np.array(adc_batches[ch_id], dtype=np.uint16),
                                np.array(tms_batches[ch_id], dtype=np.uint32)
                            )
                            adc_batches[ch_id].clear()
                            tms_batches[ch_id].clear()

                    # 成功解析一帧，指针向后跳跃整帧长度 (11字节)
                    ptr += RX_FRAME_LEN

                # 循环结束后，仅保留缓冲区中未处理的尾部残留碎片，腾出内存空间
                if ptr > 0:
                    del raw_buf[:ptr]

            except Exception as e:
                self.error_occurred.emit(f"数据串流解析异常: {str(e)}")
                break

        # 线程退出前，清空投递残余缓存，确保最后一批波形不丢失
        for ch in range(1, 5):
            if adc_batches[ch]:
                self.samples_ready.emit(
                    ch, 
                    np.array(adc_batches[ch], dtype=np.uint16), 
                    np.array(tms_batches[ch], dtype=np.uint32)
                )
        self._close_port()
    def stop(self):
        """安全终止串口接收线程，释放资源"""
        self._running = False
        # 等待线程安全退出，最多等待 2000 毫秒
        if not self.wait(2000):
            self.terminate()
            self.wait()
            self._close_port()

    def _close_port(self):
        """关闭串口连接"""
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass

# ============================================================================
#  Main Multi-Channel Scope Window (完全修复顺序与触发引擎版)
# ============================================================================

class OscilloscopeWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Digital Storage Oscilloscope v6.8 (Multi-Channel Sync)")
        self.setMinimumSize(1200, 720)

        # 【核心修复】：必须先将所有后台底层数据结构完全初始化，再加载 UI 界面
        self._t_bufs = {ch: deque(maxlen=DATA_BUF_MAX) for ch in range(1, 5)}
        self._v_bufs = {ch: deque(maxlen=DATA_BUF_MAX) for ch in range(1, 5)}
        self._ch_enabled = {1: True, 2: True, 3: False, 4: False}

        self._time_div  = DEFAULT_TIME_DIV
        self._volts_div = DEFAULT_VOLTS_DIV
        
        self._trig_mode   = "AUTO"       
        self._trig_edge   = "RISING"     
        self._trig_level  = 1.65         
        self._trig_source = 1            # 默认以 CH1 作为触发源

        self._connected   = False
        self._rx_thread: MultiChannelSerialRxThread | None = None

        # 时间戳归零基准：每个通道首个数据包的时间戳，显示时减去它以归零 X 轴
        self._t_base: dict[int, int | None] = {ch: None for ch in range(1, 5)}

        # 数据和变量准备就绪后，才可以安全地构建 UI 控件
        self._init_ui()

        # 刷新定时器 (控制波形显示帧率)
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_scope)
        self._refresh_timer.start(REFRESH_MS)

        # 端口扫描定时器
        self._scan_timer = QTimer(self)
        self._scan_timer.timeout.connect(self._scan_ports)
        self._scan_timer.start(2000)
        self._scan_ports()

    def _init_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        root = QHBoxLayout(cw)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        root.addWidget(self._build_ctrl_panel(), stretch=0)
        root.addWidget(self._build_scope_panel(), stretch=1)

        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("Ready — Configure port and click Connect")

    def _build_ctrl_panel(self) -> QWidget:
        p = QWidget()
        p.setMaximumWidth(360)
        ly = QVBoxLayout(p)
        ly.setSpacing(6)

        # 1. 串口连接配置
        grp = QGroupBox("Serial Connection")
        g = QGridLayout(grp)
        g.addWidget(QLabel("Port:"), 0, 0)
        self._combo_port = QComboBox()
        g.addWidget(self._combo_port, 0, 1)
        
        g.addWidget(QLabel("Baud:"), 1, 0)
        self._combo_baud = QComboBox()
        self._combo_baud.addItems(["115200", "460800", "921600", "2000000"])
        self._combo_baud.setCurrentText(str(DEFAULT_BAUD))
        g.addWidget(self._combo_baud, 1, 1)

        self._btn_connect = QPushButton("CONNECT")
        self._btn_connect.setMinimumHeight(36)
        self._btn_connect.clicked.connect(self._on_connect_clicked)
        self._update_connect_btn_style()
        g.addWidget(self._btn_connect, 2, 0, 1, 2)
        ly.addWidget(grp)

        # 2. 硬件通道勾选 (这里会读取 self._ch_enabled)
        grp = QGroupBox("Hardware Channels")
        g = QHBoxLayout(grp)
        self._chk_channels = {}
        for ch in range(1, 5):
            chk = QCheckBox(f"CH{ch}")
            chk.setChecked(self._ch_enabled[ch])
            chk.setStyleSheet(f"QCheckBox {{ color: {CLR_CH[ch]}; font-weight: bold; }}")
            chk.stateChanged.connect(lambda _, c=ch: self._on_channel_toggled(c))
            g.addWidget(chk)
            self._chk_channels[ch] = chk
        ly.addWidget(grp)

        # 3. 垂直档位设置
        grp = QGroupBox("Vertical (VOLTS / DIV)")
        g = QVBoxLayout(grp)
        self._volts_group = QButtonGroup(self)
        row = QHBoxLayout()
        self._volts_btns = {}
        for i, v in enumerate(VOLTS_DIV_OPTIONS):
            btn = QPushButton(self._fmt_volts(v))
            btn.setCheckable(True)
            if abs(v - DEFAULT_VOLTS_DIV) < 0.001:
                btn.setChecked(True)
            btn.clicked.connect(lambda _, val=v: self._set_volts_div(val))
            self._volts_group.addButton(btn, i)
            self._volts_btns[v] = btn
            row.addWidget(btn)
        self._style_div_btns(self._volts_btns)
        g.addLayout(row)
        ly.addWidget(grp)

        # 4. 水平时基设置
        grp = QGroupBox("Horizontal (TIME / DIV)")
        g = QVBoxLayout(grp)
        self._time_group = QButtonGroup(self)
        row1, row2 = QHBoxLayout(), QHBoxLayout()
        self._time_btns = {}
        for i, v in enumerate(TIME_DIV_OPTIONS):
            btn = QPushButton(self._fmt_time(v))
            btn.setCheckable(True)
            if abs(v - DEFAULT_TIME_DIV) < 0.0001:
                btn.setChecked(True)
            btn.clicked.connect(lambda _, val=v: self._set_time_div(val))
            self._time_group.addButton(btn, i)
            self._time_btns[v] = btn
            (row1 if i < 4 else row2).addWidget(btn)
        self._style_div_btns(self._time_btns)
        g.addLayout(row1); g.addLayout(row2)
        ly.addWidget(grp)

        # 5. 触发控制引擎面板
        grp = QGroupBox("Trigger Engine")
        g = QVBoxLayout(grp)
        
        row = QHBoxLayout()
        row.addWidget(QLabel("Source:"))
        self._combo_trig_src = QComboBox()
        self._combo_trig_src.addItems(["CH1", "CH2", "CH3", "CH4"])
        self._combo_trig_src.currentIndexChanged.connect(self._on_trig_src_changed)
        row.addWidget(self._combo_trig_src)
        
        row.addWidget(QLabel("Mode:"))
        self._btn_mode = QPushButton("AUTO (Rolling)")
        self._btn_mode.setCheckable(True)
        self._btn_mode.clicked.connect(self._on_trig_mode_toggle)
        self._btn_mode.setStyleSheet("background-color:#0078D4; color:white; font-weight:bold; border-radius:3px; min-height:24px;")
        row.addWidget(self._btn_mode)
        g.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(QLabel("Edge:"))
        self._btn_edge = QPushButton("RISING ↗")
        self._btn_edge.setCheckable(True)
        self._btn_edge.clicked.connect(self._on_trig_edge_toggle)
        self._btn_edge.setStyleSheet("background-color:#333; color:#CCC; border:1px solid #555; border-radius:3px; min-height:24px;")
        row.addWidget(self._btn_edge)
        g.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(QLabel("Level:"))
        self._slider_trig = QSlider(Qt.Horizontal)
        self._slider_trig.setRange(0, 330) 
        self._slider_trig.setValue(int(self._trig_level * 100))
        self._slider_trig.valueChanged.connect(self._on_trig_level_change)
        row.addWidget(self._slider_trig)
        self._lbl_trig_val = QLabel(f"{self._trig_level:.2f}V")
        self._lbl_trig_val.setFixedWidth(40)
        row.addWidget(self._lbl_trig_val)
        g.addLayout(row)
        ly.addWidget(grp)

        # 6. 系统监视器信息面板
        grp = QGroupBox("System Monitor")
        g = QVBoxLayout(grp)
        self._lbl_info = QLabel()
        self._lbl_info.setFont(QFont("Consolas", 9))
        self._lbl_info.setStyleSheet("color:#AAA;")
        g.addWidget(self._lbl_info)
        ly.addWidget(grp)

        ly.addStretch()
        return p

    def _build_scope_panel(self) -> QWidget:
        p = QWidget()
        ly = QVBoxLayout(p)
        ly.setContentsMargins(0, 0, 0, 0)

        self._plot = pg.PlotWidget()
        self._plot.setBackground(CLR_BG)
        
        # 4通道波形线初始化
        self._curves = {}
        for ch in range(1, 5):
            self._curves[ch] = self._plot.plot(pen=pg.mkPen(color=CLR_CH[ch], width=1.6))
            self._curves[ch].setVisible(self._ch_enabled[ch])
        
        self._trig_line = pg.InfiniteLine(angle=0, pen=pg.mkPen(color=CLR_TRIGGER, style=Qt.DashLine, width=1.5))
        self._trig_line.setValue(self._trig_level)
        self._trig_line.setVisible(False)
        self._plot.addItem(self._trig_line)

        self._plot.getPlotItem().showAxis('right', False)
        self._plot.getPlotItem().showAxis('top', False)
        
        vb = self._plot.getPlotItem().getViewBox()
        vb.setMouseEnabled(x=False, y=False)
        self._plot.setMenuEnabled(False)

        self._apply_axes()
        ly.addWidget(self._plot)
        return p

    def _on_channel_toggled(self, ch: int):
        is_checked = self._chk_channels[ch].isChecked()
        self._ch_enabled[ch] = is_checked
        self._curves[ch].setVisible(is_checked)
        if not is_checked:
            self._curves[ch].setData([], [])

    def _set_volts_div(self, v: float):
        self._volts_div = v
        self._style_div_btns(self._volts_btns)
        self._apply_axes()

    def _set_time_div(self, v: float):
        self._time_div = v
        self._style_div_btns(self._time_btns)
        self._apply_axes()

    def _apply_axes(self):
        y_half = self._volts_div * V_DIVS / 2.0
        y_mid  = VREF / 2.0
        self._plot.setYRange(y_mid - y_half, y_mid + y_half, padding=0)
        self._plot.setLabel("left", "Voltage", units="V", color="#888")
        self._plot.setLabel("bottom", "Time", units="ms", color="#888")
        self._plot.showGrid(x=True, y=True, alpha=0.15)
        
        win_ms = self._time_div * H_DIVS * 1000.0
        if self._trig_mode == "NORMAL":
            self._plot.setXRange(-win_ms * 0.1, win_ms * 0.9, padding=0)

    def _style_div_btns(self, btn_dict: dict):
        base = "QPushButton { border-radius:3px; font-size:11px; font-weight:bold; "
        sel  = base + "background-color:#0078D4; color:white; border:1px solid #005A9E; }"
        uns  = base + "background-color:#1E1E2A; color:#777; border:1px solid #333; } QPushButton:hover { background-color:#2A2A3A; color:#AAA; }"
        for btn in btn_dict.values():
            btn.setStyleSheet(sel if btn.isChecked() else uns)

    def _on_trig_mode_toggle(self, checked: bool):
        if checked:
            self._trig_mode = "NORMAL"
            self._btn_mode.setText("NORMAL (Trig'd)")
            self._btn_mode.setStyleSheet("background-color:#D46400; color:white; font-weight:bold; border-radius:3px; min-height:24px;")
            self._trig_line.setVisible(True)
        else:
            self._trig_mode = "AUTO"
            self._btn_mode.setText("AUTO (Rolling)")
            self._btn_mode.setStyleSheet("background-color:#0078D4; color:white; font-weight:bold; border-radius:3px; min-height:24px;")
            self._trig_line.setVisible(False)
        self._apply_axes()

    def _on_trig_edge_toggle(self, checked: bool):
        self._trig_edge = "FALLING" if checked else "RISING"
        self._btn_edge.setText("FALLING ↘" if checked else "RISING ↗")

    def _on_trig_src_changed(self, index: int):
        self._trig_source = index + 1

    def _on_trig_level_change(self, val: int):
        self._trig_level = val / 100.0
        self._lbl_trig_val.setText(f"{self._trig_level:.2f}V")
        self._trig_line.setValue(self._trig_level)

    def _on_connect_clicked(self):
        if self._connected:
            if self._rx_thread:
                self._rx_thread.stop()
                self._rx_thread = None
            self._connected = False
            # 重置时间基准，下次连接从零开始
            for ch in range(1, 5):
                self._t_base[ch] = None
            self._update_connect_btn_style()
            self._combo_port.setEnabled(True)
            self._combo_baud.setEnabled(True)
            self._status.showMessage("Disconnected")
        else:
            port = self._combo_port.currentText()
            baud = int(self._combo_baud.currentText())
            if not port or port.startswith("("):
                QMessageBox.warning(self, "Error", "No valid serial port selected.")
                return
            # 连接前清空旧数据和时间基准
            for ch in range(1, 5):
                self._t_bufs[ch].clear()
                self._v_bufs[ch].clear()
                self._t_base[ch] = None

            self._rx_thread = MultiChannelSerialRxThread(port, baud)
            self._rx_thread.samples_ready.connect(self._on_samples_received)
            self._rx_thread.error_occurred.connect(self._on_rx_error)
            self._rx_thread.start()
            self._connected = True
            self._update_connect_btn_style()
            self._combo_port.setEnabled(False)
            self._combo_baud.setEnabled(False)
            self._status.showMessage(f"Connected to {port} @ {baud} bps")

    def _update_connect_btn_style(self):
        if self._connected:
            self._btn_connect.setText("DISCONNECT")
            self._btn_connect.setStyleSheet("QPushButton { background-color:#D13438; color:white; font-weight:bold; border-radius:4px; font-size:14px; } QPushButton:hover { background-color:#A4262C; }")
        else:
            self._btn_connect.setText("CONNECT")
            self._btn_connect.setStyleSheet("QPushButton { background-color:#0078D4; color:white; font-weight:bold; border-radius:4px; font-size:14px; } QPushButton:hover { background-color:#106EBE; }")

    def _scan_ports(self):
        if self._connected: return
        cur = self._combo_port.currentText()
        ports = serial.tools.list_ports.comports()
        names = sorted(set(p.device for p in ports))
        self._combo_port.blockSignals(True)
        self._combo_port.clear()
        if names:
            self._combo_port.addItems(names)
            if cur in names: self._combo_port.setCurrentText(cur)
        else:
            self._combo_port.addItem("(No ports)")
        self._combo_port.blockSignals(False)

    def _on_samples_received(self, ch_id: int, adc: np.ndarray, tms: np.ndarray):
        if ch_id in self._v_bufs:
            # 首个数据包的时间戳作为该通道的零点，归零后存储
            if self._t_base[ch_id] is None and len(tms) > 0:
                self._t_base[ch_id] = int(tms[0])
            base = self._t_base[ch_id] or 0
            self._t_bufs[ch_id].extend(int(t - base) for t in tms)
            self._v_bufs[ch_id].extend(adc)

    def _on_rx_error(self, msg: str):
        self._status.showMessage(f"Serial Error: {msg}")
        if self._connected: self._on_connect_clicked() 

    def _refresh_scope(self):
        # 1. 确保触发源通道有足够的数据可以进行分析，否则不予处理
        src = self._trig_source
        if len(self._v_bufs[src]) < 50:
            return

        win_ms = self._time_div * H_DIVS * 1000.0  # 计算当前窗口总时间基准 (ms)

        # 2. 局部转存 NumPy 阵列进行高速矩阵运算，杜绝频繁读取带来的锁竞争
        t_snapshot = {}
        v_snapshot = {}
        for ch in range(1, 5):
            if self._ch_enabled[ch] and len(self._v_bufs[ch]) > 0:
                t_snapshot[ch] = np.array(self._t_bufs[ch], dtype=np.float64) / 1000.0
                v_snapshot[ch] = np.array(self._v_bufs[ch], dtype=np.float64) * VREF / ADC_MAX

        # ====================================================================
        #  模式 A：AUTO (Rolling 滚动模式) —— 独立时间轴，保持物理连续流
        # ====================================================================
        if self._trig_mode == "AUTO":
            latest_ms = -1.0
            for ch in range(1, 5):
                if ch in t_snapshot:
                    t_arr = t_snapshot[ch]
                    v_arr = v_snapshot[ch]
                    # ---- 批次空洞断线：相邻点时间差 > 阈值时插入 NaN ----
                    gaps = np.diff(t_arr) > GAP_THRESHOLD_MS
                    if np.any(gaps):
                        gap_idx = np.where(gaps)[0] + 1
                        t_arr = np.insert(t_arr, gap_idx, np.nan)
                        v_arr = np.insert(v_arr, gap_idx, np.nan)
                    # --------------------------------------------------------
                    self._curves[ch].setData(t_arr, v_arr)
                    # 取最后一个非 NaN 的时间
                    valid_t = t_arr[~np.isnan(t_arr)]
                    if len(valid_t) > 0 and valid_t[-1] > latest_ms:
                        latest_ms = valid_t[-1]
            if latest_ms > 0:
                self._plot.setXRange(max(0, latest_ms - win_ms), latest_ms, padding=0)

            info_str = "Mode: AUTO (Rolling)\n"
            for ch in range(1, 5):
                if ch in v_snapshot:
                    info_str += f"CH{ch} Buffer: {len(v_snapshot[ch])} pts | Max: {np.max(v_snapshot[ch][-200:]):.2f}V\n"
            self._lbl_info.setText(info_str)

        # ====================================================================
        #  模式 B：NORMAL (Triggered 触发模式) —— 工业级等间距连续数据切片对齐引擎
        # ====================================================================
        elif self._trig_mode == "NORMAL":
            if src not in t_snapshot:
                self._lbl_info.setText(f"Mode: NORMAL\nWaiting for source CH{src} data...")
                return

            t_src = t_snapshot[src]
            v_src = v_snapshot[src]
            lvl = self._trig_level

            # 预估算每毫秒点数，用于确定搜索窗和显示窗大小
            pts_per_ms_est = len(v_src) / (t_src[-1] - t_src[0] + 1e-6)
            post_pts_needed = int(win_ms * 0.9 * pts_per_ms_est)
            pre_pts_needed  = int(win_ms * 0.1 * pts_per_ms_est)

            # 搜索窗从末尾往前挪 post_pts_needed，确保触发点后有足够的显示数据
            search_end   = len(v_src) - post_pts_needed
            search_start = max(0, search_end - max(3000, pre_pts_needed + post_pts_needed))
            if search_end <= search_start:
                # 缓冲区数据还不够填满一屏
                return

            v_look = v_src[search_start:search_end]
            t_look = t_src[search_start:search_end]

            # —— 利用 NumPy 硬件级向量化矩阵操作高速捕捉跳变沿 ——
            if self._trig_edge == "RISING":
                condition = (v_look[:-1] < lvl) & (v_look[1:] >= lvl)
            else:
                condition = (v_look[:-1] > lvl) & (v_look[1:] <= lvl)

            trig_indices = np.where(condition)[0]

            if trig_indices.size > 0:
                # 锚定最晚的触发边沿
                idx = trig_indices[-1]

                # 时间戳亚采样插值校准
                t0, t1 = t_look[idx], t_look[idx+1]
                v0, v1 = v_look[idx], v_look[idx+1]
                if abs(v1 - v0) > 1e-6:
                    fraction = (lvl - v0) / (v1 - v0)
                    trig_time = t0 + fraction * (t1 - t0)
                else:
                    trig_time = t1

                # 触发点在完整缓冲区中的绝对索引
                trig_abs_idx = search_start + idx + 1

                # 联动刷新所有被选中的使能通道
                for ch in range(1, 5):
                    if ch in t_snapshot and ch in v_snapshot:
                        ch_t = t_snapshot[ch]
                        ch_v = v_snapshot[ch]

                        # 推算其他通道的等效触发索引
                        ch_trig_idx = trig_abs_idx + (len(ch_v) - len(v_src))
                        if ch_trig_idx <= 0 or ch_trig_idx >= len(ch_v):
                            continue

                        start_idx = max(0, ch_trig_idx - pre_pts_needed)
                        end_idx   = min(len(ch_v), ch_trig_idx + post_pts_needed)

                        # 剥离相对时间原点，确保多通道波形在屏幕上横轴完全对齐
                        t_seg = ch_t[start_idx:end_idx] - trig_time
                        v_seg = ch_v[start_idx:end_idx]

                        # ---- 批次空洞断线 ----
                        gaps = np.diff(t_seg) > GAP_THRESHOLD_MS
                        if np.any(gaps):
                            gap_idx = np.where(gaps)[0] + 1
                            t_seg = np.insert(t_seg, gap_idx, np.nan)
                            v_seg = np.insert(v_seg, gap_idx, np.nan)
                        # --------------------

                        self._curves[ch].setData(t_seg, v_seg)

                self._lbl_info.setText(
                    f"Mode: NORMAL (Trig'd on CH{src})\n"
                    f"Trig Level: {lvl:.2f}V | Edge: {self._trig_edge}\n"
                    f"Render Pts: {end_idx - start_idx} | Status: Synchronized"
                )
            else:
                self._lbl_info.setText(f"Mode: NORMAL (Hold/Wait)\nSearching valid edge on CH{src}...")

    @staticmethod
    def _fmt_volts(v: float) -> str: return f"{v:.0f}V" if v >= 1 else f"{v:.1f}V"
    @staticmethod
    def _fmt_time(v: float) -> str: return f"{v*1000:.0f}ms" if v >= 0.001 else f"{v*1e6:.0f}μs"

    def closeEvent(self, event):
        self._refresh_timer.stop()
        self._scan_timer.stop()
        if self._rx_thread and self._rx_thread.isRunning():
            self._rx_thread.stop()
        event.accept()

def main():
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    font = QFont(); font.setFamilies(["Segoe UI", "Consolas", "sans-serif"])
    font.setPointSize(9); app.setFont(font)

    # Safe stylesheet composition bypassing PyQt style parsing bugs
    style_str = "QMainWindow { background-color:" + CLR_BG + "; }\n"
    style_str += "QGroupBox { font-weight:bold; border:1px solid #3A3A4A; border-radius:4px; margin-top:6px; padding-top:12px; color:#BBB; }\n"
    style_str += "QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; color:#999; }\n"
    style_str += "QLabel { color:#AAA; }\n"
    style_str += "QComboBox { background:#1C1C28; color:#CCC; border:1px solid #444; padding:2px 6px; border-radius:3px; }\n"
    style_str += "QComboBox QAbstractItemView { background:#1C1C28; color:#CCC; selection-background-color:#0078D4; }\n"
    style_str += "QSlider::groove:horizontal { border:1px solid #444; height:6px; background:#2A2A38; border-radius:3px; }\n"
    style_str += "QSlider::handle:horizontal { background:" + CLR_TRIGGER + "; border:1px solid #CC2222; width:14px; margin:-4px 0; border-radius:7px; }\n"
    style_str += "QStatusBar { color:#666; background:#0C0C14; border-top:1px solid #1A1A2A; }"
    app.setStyleSheet(style_str)

    window = OscilloscopeWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()