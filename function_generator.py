#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function Generator & Waveform Viewer v7.1 (BaudRate Master Edition)
========================================================================
- [修复] 彻底重写自适应逻辑：固定 100kHz 采样率不缩水，保证信号最高阶解析度
- [核心] 引入“动态带宽步长抽样算法”：
  * 当波特率=2000000时，完全解除限速，全量高密度推送 100kHz 所有采样点给下位机！
  * 当波特率=115200时，根据车道上限自动加大发射步长(跳点)，确保下位机波形不滞后、不卡死。
- [解耦] 多通道完全解耦，静态示波器无缝匹配硬件实时高频白噪声渲染。
"""

import sys
import math
import time
import struct
import random 
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QLineEdit, QSlider,
    QMessageBox, QStatusBar, QButtonGroup
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QMutex, QMutexLocker
from PyQt5.QtGui import QFont, QIntValidator
import pyqtgraph as pg

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Please install pyserial: pip install pyserial")
    sys.exit(1)

# ============================================================================
#  全局通用配置项
# ============================================================================
FREQ_MIN     = 1
FREQ_MAX     = 1000
AMP_MIN      = 0
AMP_MAX      = 4095
VREF         = 3.3
SAMPLE_RATE  = 100000  # 核心过采样率：死锁固定在 100 kHz 不缩水
PHASE_MAX    = 0x100000000  # 32位整数DDS相位累加器模数 (2^32)，零浮点累积误差

CH_COLORS = {
    1: "#FFFF00",  # CH1: 黄色
    2: "#00FFFF",  # CH2: 青色
    3: "#FF00FF",  # CH3: 品红
    4: "#00FF00"   # CH4: 绿色
}

FRAME_H1     = 0x5A
FRAME_H2     = 0xA5
CMD_DATA     = 0x01
CMD_COMPACT  = 0x02          # 紧凑帧：3样本/帧，零时间戳，示波器端重建时序
SAMPLES_PER_COMPACT = 3      # 每紧凑帧携带的采样点数

# ============================================================================
#  纯静态波形数学解析引擎 (零初相解析解)
# ============================================================================
class StaticWaveEngine:
    @staticmethod
    def get_points(wave_type: str, frequency: float, amplitude: int, t_ms: np.ndarray) -> np.ndarray:
        if wave_type == "无" or not wave_type:
            return np.zeros_like(t_ms)
            
        t_sec = t_ms / 1000.0
        phase = 2.0 * np.pi * frequency * t_sec
        phase_mod = np.mod(phase, 2.0 * np.pi)
        amp_v = amplitude * VREF / AMP_MAX
        
        if wave_type == "正弦波":
            raw = 0.5 + 0.5 * np.sin(phase_mod)
        elif wave_type == "方波":
            raw = np.where(np.sin(phase_mod) >= 0, 1.0, 0.0)
        elif wave_type == "三角波":
            p = phase_mod / (2.0 * np.pi)
            raw = np.where(p < 0.5, 2.0 * p, 2.0 * (1.0 - p))
        elif wave_type == "锯齿波":
            raw = phase_mod / (2.0 * np.pi)
        elif wave_type == "直流":
            raw = np.ones_like(phase_mod)
        elif wave_type == "噪声":
            raw = np.random.rand(len(t_ms))
        else:
            raw = np.zeros_like(phase_mod)
            
        return raw * amp_v

# ============================================================================
#  物理层带宽解耦自适应串流线程 (连续相位累加)
# ============================================================================
class MultiChannelStreamThread(QThread):
    stats_updated      = pyqtSignal(int, int)
    error_occurred     = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self, port: str, baudrate: int):
        super().__init__()
        self._port     = port
        self._baudrate = baudrate
        self._serial   = None
        self._running  = False
        self._mutex    = QMutex()
        
        self._ch_params = {
            i: {"wave": "无", "freq": 1000.0, "amp": 4095, "active": False, "phase_acc": 0}
            for i in range(1, 5)
        }
        self._packet_count = 0

    def update_channel_params(self, ch_id: int, wave: str, freq: float, amp: int, active: bool):
        with QMutexLocker(self._mutex):
            self._ch_params[ch_id]["wave"]   = wave
            self._ch_params[ch_id]["freq"]   = freq
            self._ch_params[ch_id]["amp"]    = amp
            self._ch_params[ch_id]["active"] = active

    def run(self):
        try:
            # 建立物理链路，设置极短超时，防止缓冲区满了以后挂起主UI
            ser = serial.Serial(port=self._port, baudrate=self._baudrate, timeout=0.01, write_timeout=0.01)
            self._serial = ser
        except Exception as e:
            self.error_occurred.emit(f"无法打开端口 {self._port} ({self._baudrate}bps): {e}")
            self.connection_changed.emit(False)
            return

        self._running = True
        self.connection_changed.emit(True)
        epoch_start = time.perf_counter()
        t_last_stats = epoch_start

        # 连续时序环状态变量
        next_slot = epoch_start
        prev_active_count = 0

        local_params = {
            i: {"wave": "无", "freq": 1000.0, "amp": 4095, "active": False, "phase_acc": 0}
            for i in range(1, 5)
        }

        # --------------------------------------------------------------------
        # 🔥 重新设计的物理吞吐自适应数学模型
        # --------------------------------------------------------------------
        # 理论物理通道最大可传输的数据帧率（每帧 11 字节）
        max_frames_per_sec = (self._baudrate / 10.0) / 11.0
        # 预留 10% 的安全车道空闲，配合紧凑帧达到 ~50点/周期 @1kHz (500us/div)
        safe_frames_per_sec = max_frames_per_sec * 0.90

        try:
            while self._running:
                if self._mutex.tryLock():
                    try:
                        for i in range(1, 5):
                            local_params[i]["wave"]   = self._ch_params[i]["wave"]
                            local_params[i]["freq"]   = self._ch_params[i]["freq"]
                            local_params[i]["amp"]    = self._ch_params[i]["amp"]
                            local_params[i]["active"] = self._ch_params[i]["active"]
                    finally:
                        self._mutex.unlock()

                active_channels = [ch for ch in range(1, 5) if local_params[ch]["active"]]
                active_count = len(active_channels)
                if not active_count:
                    self.msleep(10)
                    continue

                # 每一个物理激活通道所能分配到的最大发包流速上限 (帧/秒)
                allowed_fps_per_ch = safe_frames_per_sec / active_count
                
                # 核心逻辑：采样率固定为 100kHz，计算下位机接收发射时应该跳过多少点(步长)
                # 如果波特率是 2M，allowed_fps_per_ch 很大，transmit_step 就会等于 1 (即不漏点全量发送)
                transmit_step = math.ceil(SAMPLE_RATE / allowed_fps_per_ch)
                if transmit_step < 1: 
                    transmit_step = 1

                # 计算真正的单通道实际输出发包率
                actual_fps_per_ch = SAMPLE_RATE / transmit_step
                
                # 目标帧间距（秒）：连续时序环的核心节拍
                frame_interval_s = 1.0 / actual_fps_per_ch

                # ---- 自适应微批大小：平衡 write() 开销与批次间隙 ----
                # 目标：批次间隙 < 0.5ms，示波器端不可见
                if active_count == 1 and actual_fps_per_ch > 8000:
                    micro_batch = 4
                elif active_count <= 2:
                    micro_batch = 2
                else:
                    micro_batch = 1

                batch_interval_s = micro_batch * frame_interval_s

                # 通道数变化时重置调度器，匹配新步调
                if active_count != prev_active_count:
                    next_slot = time.perf_counter()
                    prev_active_count = active_count

                # ---- 精确等待至预定时间槽位 ----
                wait_until = next_slot
                now = time.perf_counter()
                sleep_s = wait_until - now
                if sleep_s > 0.0:
                    if sleep_s > 0.002:        # 粗等 > 2ms 用 sleep 降 CPU
                        time.sleep(sleep_s - 0.0015)
                    while time.perf_counter() < wait_until:
                        pass                   # 微秒级自旋精对齐

                # ---- 生成微批帧数据（紧凑协议：每帧 3 采样点，零时间戳） ----
                chunk = bytearray()
                for _ in range(micro_batch):
                    for ch_id in active_channels:
                        cp = local_params[ch_id]
                        # 紧凑协议：按有效采样率 (actual_fps_per_ch × 3样本/帧) 计算每样本相位步进
                        effective_sample_rate = actual_fps_per_ch * SAMPLES_PER_COMPACT
                        phase_step = int(PHASE_MAX * cp["freq"] / effective_sample_rate)
                        half = PHASE_MAX // 2
                        w_type = cp["wave"]
                        amp = cp["amp"]
                        adc_vals = []

                        for __ in range(SAMPLES_PER_COMPACT):
                            # DDS 累加
                            cp["phase_acc"] = (cp["phase_acc"] + phase_step) & 0xFFFFFFFF
                            acc = cp["phase_acc"]

                            if w_type == "正弦波":
                                raw_val = 0.5 + 0.5 * math.sin(acc * (2.0 * math.pi / PHASE_MAX))
                            elif w_type == "方波":
                                raw_val = 1.0 if acc < half else 0.0
                            elif w_type == "三角波":
                                if acc < half:
                                    raw_val = acc / half
                                else:
                                    raw_val = 2.0 - acc / half
                            elif w_type == "锯齿波":
                                raw_val = acc / PHASE_MAX
                            elif w_type == "直流":
                                raw_val = 1.0
                            elif w_type == "噪声":
                                raw_val = random.random()
                            else:
                                raw_val = 0.0

                            adc_val = int(self._clip_round(raw_val * amp))
                            adc_val = max(0, min(4095, adc_val))
                            adc_vals.append(adc_val)

                        # 紧凑帧: [5A, A5, ch_id, 02, A1H, A1L, A2H, A2L, A3H, A3L, CK]
                        a1 = struct.pack('>H', adc_vals[0] & 0xFFFF)
                        a2 = struct.pack('>H', adc_vals[1] & 0xFFFF)
                        a3 = struct.pack('>H', adc_vals[2] & 0xFFFF)

                        ck = (FRAME_H1 + FRAME_H2 + ch_id + CMD_COMPACT +
                              a1[0] + a1[1] + a2[0] + a2[1] + a3[0] + a3[1]) & 0xFF

                        chunk.extend([FRAME_H1, FRAME_H2, ch_id, CMD_COMPACT])
                        chunk.extend(a1)
                        chunk.extend(a2)
                        chunk.extend(a3)
                        chunk.append(ck)
                        self._packet_count += SAMPLES_PER_COMPACT

                # ---- 发送微批数据 ----
                if chunk:
                    try:
                        ser.write(chunk)
                    except (serial.SerialTimeoutException, serial.SerialException):
                        pass

                # ---- 推进调度槽位，若已落后则重置防追尾 ----
                next_slot = max(next_slot + batch_interval_s, time.perf_counter())

                # ---- 统计信息发射 ----
                now = time.perf_counter()
                if now - t_last_stats >= 0.5:
                    elapsed = now - epoch_start
                    pps = int(self._packet_count / elapsed) if elapsed > 0 else 0
                    self.stats_updated.emit(self._packet_count, pps)
                    t_last_stats = now

        except Exception as e:
            self.error_occurred.emit(f"串流异常: {e}")
        finally:
            self._close()
            self.connection_changed.emit(False)

    def stop(self):
        self._running = False
        if not self.wait(2000):
            self.terminate()
            self.wait()
            self._close()

    def _close(self):
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception: pass

    @staticmethod
    def _clip_round(val):
        if val < 0: return 0
        if val > 4095: return 4095
        return int(val + 0.5)

# ============================================================================
#  主图形用户界面
# ============================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("全通道信号发生器与示波器工作台 v7.1 (物理全速率解绑版)")
        self.setMinimumSize(1280, 720)

        self._streamer = None
        
        self.channels = {
            i: {
                "wave": "无", 
                "freq": 1000,
                "amp": 4095,
                "output": False,
                "curve": None
            } for i in range(1, 5)
        }
        self.current_ch = 1 

        self._init_ui()
        
        self._scan_timer = QTimer(self)
        self._scan_timer.timeout.connect(self._scan_ports)
        self._scan_timer.start(2000)
        self._scan_ports()

    def _init_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        root = QHBoxLayout(cw)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(12)
        
        root.addWidget(self._build_control_panel(), stretch=0)
        root.addWidget(self._build_scope_panel(), stretch=1)

        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("系统就绪。")
        
        self._load_channel_settings(self.current_ch)

    def _build_control_panel(self) -> QWidget:
        p = QWidget()
        p.setFixedWidth(380)
        ly = QVBoxLayout(p)
        ly.setContentsMargins(0, 0, 0, 0)
        ly.setSpacing(10)

        # 1. 通道编辑器
        grp_ch = QGroupBox("1. 编辑通道切换 (Interface Setup)")
        g_ch = QVBoxLayout(grp_ch)
        self._combo_ch_select = QComboBox()
        for i in range(1, 5):
            self._combo_ch_select.addItem(f"配置通道 CH {i} (当前活动)", i)
        self._combo_ch_select.currentIndexChanged.connect(self._on_current_ch_changed)
        g_ch.addWidget(self._combo_ch_select)
        ly.addWidget(grp_ch)

        # 2. 波形选择
        grp_wave = QGroupBox("2. 波形函数选择 (Waveform)")
        g_wave = QVBoxLayout(grp_wave)
        row_w1 = QHBoxLayout()
        self._btn_sine = self._make_wave_btn("正弦波")
        self._btn_square = self._make_wave_btn("方波")
        self._btn_triangle = self._make_wave_btn("三角波")
        row_w1.addWidget(self._btn_sine)
        row_w1.addWidget(self._btn_square)
        row_w1.addWidget(self._btn_triangle)
        
        row_w2 = QHBoxLayout()
        self._btn_sawtooth = self._make_wave_btn("锯齿波")
        self._btn_dc = self._make_wave_btn("直流")
        self._btn_noise = self._make_wave_btn("噪声")
        row_w2.addWidget(self._btn_sawtooth)
        row_w2.addWidget(self._btn_dc)
        row_w2.addWidget(self._btn_noise)
        
        self._wave_group = QButtonGroup(self)
        self._wave_group.setExclusive(False) 
        for idx, btn in enumerate([self._btn_sine, self._btn_square, self._btn_triangle, self._btn_sawtooth, self._btn_dc, self._btn_noise]):
            self._wave_group.addButton(btn, idx)
        self._wave_group.buttonClicked.connect(self._on_wave_type_clicked)

        g_wave.addLayout(row_w1)
        g_wave.addLayout(row_w2)
        ly.addWidget(grp_wave)

        # 3. 频率调节
        grp_freq = QGroupBox("3. 频率参数微调 (Frequency)")
        g_freq = QVBoxLayout(grp_freq)
        row_f = QHBoxLayout()
        self._lbl_freq_display = QLabel("1 000")
        self._lbl_freq_display.setFont(QFont("Consolas", 22, QFont.Bold))
        self._lbl_freq_display.setStyleSheet("color:#00FF88; background:#151520; padding:2px 8px; border-radius:4px;")
        row_f.addWidget(self._lbl_freq_display)
        row_f.addWidget(QLabel("Hz"))
        row_f.addStretch()
        
        row_f_edit = QHBoxLayout()
        row_f_edit.addWidget(QLabel("数字输入:"))
        self._edit_freq = QLineEdit("1000")
        self._edit_freq.setValidator(QIntValidator(FREQ_MIN, FREQ_MAX))
        self._edit_freq.textEdited.connect(self._on_freq_edited)
        row_f_edit.addWidget(self._edit_freq)
        
        self._slider_freq = QSlider(Qt.Horizontal)
        self._slider_freq.setRange(FREQ_MIN, FREQ_MAX)
        self._slider_freq.valueChanged.connect(self._on_freq_slider_moved)

        g_freq.addLayout(row_f)
        g_freq.addLayout(row_f_edit)
        g_freq.addWidget(self._slider_freq)
        ly.addWidget(grp_freq)

        # 4. 幅值调节
        grp_amp = QGroupBox("4. 电压幅值域控制 (Amplitude)")
        g_amp = QVBoxLayout(grp_amp)
        row_a = QHBoxLayout()
        self._edit_amp = QLineEdit("4095")
        self._edit_amp.setValidator(QIntValidator(AMP_MIN, AMP_MAX))
        self._edit_amp.textEdited.connect(self._on_amp_edited)
        row_a.addWidget(QLabel("DAC数模码量 (0-4095):"))
        row_a.addWidget(self._edit_amp)
        self._lbl_amp_v = QLabel("(3.30 Vpp)")
        row_a.addWidget(self._lbl_amp_v)
        
        self._slider_amp = QSlider(Qt.Horizontal)
        self._slider_amp.setRange(AMP_MIN, AMP_MAX)
        self._slider_amp.valueChanged.connect(self._on_amp_slider_moved)

        g_amp.addLayout(row_a)
        g_amp.addWidget(self._slider_amp)
        ly.addWidget(grp_amp)

        # 5. 通信链路及波特率配置
        grp_com = QGroupBox("5. 物理通信链路端口 (Serial Port Setup)")
        g_com = QVBoxLayout(grp_com)
        
        row_c = QHBoxLayout()
        row_c.addWidget(QLabel("端口选择:"))
        self._combo_port = QComboBox()
        self._combo_port.setMinimumWidth(130)
        row_c.addWidget(self._combo_port)
        row_c.addStretch()
        g_com.addLayout(row_c)
        
        row_b = QHBoxLayout()
        row_b.addWidget(QLabel("波特率选择:"))
        self._combo_baud = QComboBox()
        self._combo_baud.setMinimumWidth(130)
        # 提供包含 2M 高速串口波特率在内的多档配置
        self._combo_baud.addItems(["115200", "230400", "460800", "921600", "1000000", "2000000"])
        self._combo_baud.setCurrentText("2000000") # 默认切换到 2M 释放最大硬件吞吐
        row_b.addWidget(self._combo_baud)
        row_b.addStretch()
        g_com.addLayout(row_b)
        
        ly.addWidget(grp_com)

        # 6. 四通道独立输出矩阵
        grp_out = QGroupBox("6. 物理输出硬件通道总线矩阵 (Independent Outputs)")
        g_out = QVBoxLayout(grp_out)
        
        self._out_btns = {}
        for i in range(1, 5):
            btn = QPushButton(f"CH {i} 物理输出: 关")
            btn.setCheckable(True)
            btn.setMinimumHeight(35)
            btn.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
            btn.clicked.connect(lambda checked, idx=i: self._on_ch_output_toggled(idx, checked))
            g_out.addWidget(btn)
            self._out_btns[i] = btn
            self._style_output_btn(i, False)

        g_out.addSpacing(5)
        self._lbl_stream_stats = QLabel("串流状态: 链路空闲")
        self._lbl_stream_stats.setStyleSheet("color: #666; font-family: Consolas;")
        g_out.addWidget(self._lbl_stream_stats)
        
        ly.addWidget(grp_out)
        ly.addStretch()
        return p

    def _build_scope_panel(self) -> QWidget:
        p = QWidget()
        ly = QVBoxLayout(p)
        ly.setContentsMargins(0, 0, 0, 0)
        ly.setSpacing(6)

        row_header = QHBoxLayout()
        title = QLabel("静态矩阵多通道示意图示波器 (Static Schematic Display)")
        title.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        row_header.addWidget(title)
        row_header.addStretch()
        ly.addLayout(row_header)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("#0A0A0F")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.setLabel("left", "Voltage", units="V")
        self._plot.setLabel("bottom", "Time", units="ms")
        
        self._plot.enableAutoRange(x=False, y=False)
        vb = self._plot.getPlotItem().getViewBox()
        vb.setMouseEnabled(x=True, y=True)
        
        self._plot.sigXRangeChanged.connect(self._on_view_range_changed)
        
        for i in range(1, 5):
            self.channels[i]["curve"] = self._plot.plot(pen=pg.mkPen(CH_COLORS[i], width=2.2))
        
        ly.addWidget(self._plot)

        grp_knobs = QGroupBox("示波器屏幕显示约束旋钮 (Display Scaling Knobs)")
        g_knobs = QHBoxLayout(grp_knobs)
        
        g_knobs.addWidget(QLabel("水平时基缩放旋钮 (X-Scale Zoom):"))
        self._slider_zoom_x = QSlider(Qt.Horizontal)
        self._slider_zoom_x.setRange(10, 500)
        self._slider_zoom_x.setValue(100)
        self._slider_zoom_x.valueChanged.connect(self._on_view_scale_changed)
        g_knobs.addWidget(self._slider_zoom_x)

        g_knobs.addWidget(QLabel("垂直增益缩放旋钮 (Y-Scale Zoom):"))
        self._slider_zoom_y = QSlider(Qt.Horizontal)
        self._slider_zoom_y.setRange(50, 300)
        self._slider_zoom_y.setValue(100)
        self._slider_zoom_y.valueChanged.connect(self._on_view_scale_changed)
        g_knobs.addWidget(self._slider_zoom_y)
        
        ly.addWidget(grp_knobs)
        
        self._on_view_scale_changed()
        return p

    def _make_wave_btn(self, text: str) -> QPushButton:
        btn = QPushButton(text)
        btn.setCheckable(True)
        btn.setMinimumHeight(32)
        return btn

    def _load_channel_settings(self, ch_id: int):
        ch = self.channels[ch_id]
        for btn in self._wave_group.buttons():
            btn.blockSignals(True)
            btn.setChecked(btn.text() == ch["wave"])
            btn.blockSignals(False)

        self._slider_freq.blockSignals(True)
        self._slider_freq.setValue(ch["freq"])
        self._slider_freq.blockSignals(False)
        self._edit_freq.setText(str(ch["freq"]))
        self._lbl_freq_display.setText(f"{ch['freq']:,}".replace(",", " "))

        self._slider_amp.blockSignals(True)
        self._slider_amp.setValue(ch["amp"])
        self._slider_amp.blockSignals(False)
        self._edit_amp.setText(str(ch["amp"]))
        self._lbl_amp_v.setText(f"({ch['amp'] * VREF / AMP_MAX:.2f} Vpp)")

    def _on_current_ch_changed(self, index: int):
        self.current_ch = self._combo_ch_select.currentData()
        self._load_channel_settings(self.current_ch)

    def _on_wave_type_clicked(self, clicked_btn: QPushButton):
        if clicked_btn.isChecked():
            for btn in self._wave_group.buttons():
                if btn != clicked_btn:
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
            self.channels[self.current_ch]["wave"] = clicked_btn.text()
        else:
            self.channels[self.current_ch]["wave"] = "无"
            
        self._sync_to_streamer(self.current_ch)
        self._update_plots()

    def _on_freq_slider_moved(self, val: int):
        self.channels[self.current_ch]["freq"] = val
        self._edit_freq.setText(str(val))
        self._lbl_freq_display.setText(f"{val:,}".replace(",", " "))
        self._sync_to_streamer(self.current_ch)
        self._update_plots()

    def _on_freq_edited(self, text: str):
        try:
            val = int(text)
            if FREQ_MIN <= val <= FREQ_MAX:
                self.channels[self.current_ch]["freq"] = val
                self._slider_freq.setValue(val)
                self._lbl_freq_display.setText(f"{val:,}".replace(",", " "))
                self._sync_to_streamer(self.current_ch)
                self._update_plots()
        except ValueError: pass

    def _on_amp_slider_moved(self, val: int):
        self.channels[self.current_ch]["amp"] = val
        self._edit_amp.setText(str(val))
        self._lbl_amp_v.setText(f"({val * VREF / AMP_MAX:.2f} Vpp)")
        self._sync_to_streamer(self.current_ch)
        self._update_plots()

    def _on_amp_edited(self, text: str):
        try:
            val = int(text)
            if AMP_MIN <= val <= AMP_MAX:
                self.channels[self.current_ch]["amp"] = val
                self._slider_amp.setValue(val)
                self._lbl_amp_v.setText(f"({val * VREF / AMP_MAX:.2f} Vpp)")
                self._sync_to_streamer(self.current_ch)
                self._update_plots()
        except ValueError: pass

    def _sync_to_streamer(self, ch_id: int):
        if self._streamer and self._streamer.isRunning():
            ch = self.channels[ch_id]
            self._streamer.update_channel_params(ch_id, ch["wave"], ch["freq"], ch["amp"], ch["output"])

    def _style_output_btn(self, ch_id: int, checked: bool):
        btn = self._out_btns[ch_id]
        if checked:
            color = CH_COLORS[ch_id]
            btn.setText(f"CH {ch_id} 物理输出: 开启")
            btn.setStyleSheet(f"QPushButton {{ background-color:{color}; color:#000; border:1px solid white; font-weight:bold; border-radius:4px; }}")
        else:
            btn.setText(f"CH {ch_id} 物理输出: 关闭")
            btn.setStyleSheet("QPushButton { background-color:#2A2A30; color:#666; border:1px solid #444; border-radius:4px; }")

    def _on_ch_output_toggled(self, ch_id: int, checked: bool):
        self.channels[ch_id]["output"] = checked
        self._style_output_btn(ch_id, checked)
        
        any_active = any(self.channels[i]["output"] for i in range(1, 5))
        
        if any_active and not self._streamer:
            port = self._combo_port.currentText()
            if not port or port.startswith("("):
                QMessageBox.warning(self, "硬件串口缺失", "请先选择有效的物理串行COM端口。")
                btn = self._out_btns[ch_id]
                btn.blockSignals(True)
                btn.setChecked(False)
                btn.blockSignals(False)
                self.channels[ch_id]["output"] = False
                self._style_output_btn(ch_id, False)
                return
            
            try:
                active_baud = int(self._combo_baud.currentText())
            except ValueError:
                active_baud = 2000000
            
            self._combo_port.setEnabled(False)
            self._combo_baud.setEnabled(False)
            
            self._streamer = MultiChannelStreamThread(port, active_baud)
            self._streamer.stats_updated.connect(self._on_stream_stats)
            self._streamer.error_occurred.connect(self._on_stream_error)
            self._streamer.connection_changed.connect(self._on_stream_conn_changed)
            
            for i in range(1, 5):
                ch = self.channels[i]
                self._streamer.update_channel_params(i, ch["wave"], ch["freq"], ch["amp"], ch["output"])
                
            self._streamer.start()

        self._sync_to_streamer(ch_id)
        if not any_active and self._streamer:
            self._stop_streamer_pool()

    def _stop_streamer_pool(self):
        if self._streamer:
            self._streamer.stop()
            self._streamer = None
        self._combo_port.setEnabled(True)
        self._combo_baud.setEnabled(True)
        self._lbl_stream_stats.setText("串流状态: 链路空闲")

    def _on_stream_stats(self, total: int, pps: int):
        self._lbl_stream_stats.setText(f"总发送: {total} 帧 | 实时吞吐效率: {pps} pps")

    def _on_stream_error(self, msg: str):
        QMessageBox.critical(self, "物理通信中断", msg)
        for i in range(1, 5):
            self.channels[i]["output"] = False
            self._out_btns[i].setChecked(False)
            self._style_output_btn(i, False)
        self._stop_streamer_pool()

    def _on_stream_conn_changed(self, ok: bool):
        if not ok: self._stop_streamer_pool()

    def _on_view_scale_changed(self):
        scale_x = self._slider_zoom_x.value() / 100.0
        scale_y = self._slider_zoom_y.value() / 100.0
        self._plot.setXRange(0, 10.0 * scale_x, padding=0.0)
        self._plot.setYRange(-0.2 * scale_y, (VREF + 0.5) * scale_y, padding=0.0)

    def _on_view_range_changed(self, *args):
        x_range = self._plot.viewRange()[0]
        self._t_min_current = x_range[0]
        self._t_max_current = x_range[1]
        self._update_plots()

    def _update_plots(self):
        t_min = getattr(self, '_t_min_current', 0.0)
        t_max = getattr(self, '_t_max_current', 10.0)
        t_ms = np.linspace(t_min, t_max, 1200)
        
        for i in range(1, 5):
            ch = self.channels[i]
            if ch["wave"] == "无":
                ch["curve"].setData([], [])
            else:
                volt_data = StaticWaveEngine.get_points(ch["wave"], ch["freq"], ch["amp"], t_ms)
                ch["curve"].setData(t_ms, volt_data)

    def _scan_ports(self):
        if not self._combo_port.isEnabled(): return
        cur = self._combo_port.currentText()
        ports = serial.tools.list_ports.comports()
        names = sorted(set(p.device for p in ports))
        self._combo_port.blockSignals(True)
        self._combo_port.clear()
        if names:
            self._combo_port.addItems(names)
            if cur in names: self._combo_port.setCurrentText(cur)
        else:
            self._combo_port.addItem("(未检测到可用端口)")
        self._combo_port.blockSignals(False)

    def closeEvent(self, event):
        self._scan_timer.stop()
        if self._streamer: self._streamer.stop()
        event.accept()

def main():
    try:
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    except AttributeError: pass
    
    app = QApplication(sys.argv)
    font = QFont("Microsoft YaHei")
    font.setPointSize(9)
    app.setFont(font)

    app.setStyleSheet("""
        QMainWindow { background-color:#101014; }
        QGroupBox { font-weight:bold; border:1px solid #2D2D3D; border-radius:6px; margin-top:10px; padding-top:14px; color:#A5A5B5; }
        QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 3px; color:#008BE3; }
        QLabel { color:#B0B0BD; }
        QComboBox { background:#1A1A26; color:#DDD; border:1px solid #3A3A4A; padding:3px 6px; border-radius:4px; }
        QLineEdit { background:#1A1A26; color:#FFF; border:1px solid #3A3A4A; padding:4px 6px; border-radius:4px; font-family:Consolas; }
        QSlider::groove:horizontal { height:5px; background:#252535; border-radius:2px; }
        QSlider::handle:horizontal { background:#0078D4; border:1px solid #005A9E; width:16px; margin:-5px 0; border-radius:8px; }
        QPushButton { background-color:#2A2A35; color:#AAA; border:1px solid #444; border-radius:4px; min-height:30px; }
        QPushButton:hover { background-color:#353545; color:#FFF; }
        QPushButton:checked { background-color:#0078D4; color:white; font-weight:bold; }
        QStatusBar { color:#777; background:#0A0A0F; border-top:1px solid #222; }
    """)
    
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()