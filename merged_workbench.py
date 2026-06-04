#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified Signal Workbench v2.0 — Function Generator + Digital Oscilloscope
=========================================================================
- Tab 1: 4-Channel Function Generator (signal output via serial, DDS with 256-pt LUT)
- Tab 2: 4-Channel Digital Storage Oscilloscope (signal capture via serial)
- Shared serial port: single serial.Serial instance, full-duplex TX+RX
- Frame format (11-byte compact): [5A A5 CH 02 A1H A1L A2H A2L A3H A3L CK]
- MCU feedback: ECHO frames (CMD=0x03) + status lines (S:...)
"""

import sys
import math
import time
import struct
import numpy as np
from collections import deque
from typing import Optional

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QLineEdit, QSlider,
    QMessageBox, QStatusBar, QButtonGroup, QCheckBox, QTabWidget
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
#  Shared Constants
# ============================================================================
FREQ_MIN     = 1
FREQ_MAX     = 1000
AMP_MIN      = 0
AMP_MAX      = 4095
VREF         = 3.3
SAMPLE_RATE  = 100000       # 固定 100 kHz 核心采样率
PHASE_MAX    = 0x100000000  # 2^32 DDS 相位累加器模数

CH_COLORS = {
    1: "#FFFF00",  # CH1: Yellow
    2: "#00FFFF",  # CH2: Cyan
    3: "#FF00FF",  # CH3: Magenta
    4: "#00FF00"   # CH4: Green
}

FRAME_H1     = 0x5A
FRAME_H2     = 0xA5
CMD_DATA     = 0x01    # 标准帧：1 样本 + 4 字节时间戳
CMD_COMPACT  = 0x02    # 紧凑帧：3 样本/帧，零时间戳
CMD_ECHO     = 0x03    # MCU 回传帧
FRAME_LEN    = 11
SAMPLES_PER_COMPACT = 3

# Oscilloscope constants
H_DIVS       = 10
V_DIVS       = 8
TIME_DIV_OPTIONS  = [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1]
VOLTS_DIV_OPTIONS = [0.1, 0.2, 0.5, 1.0, 2.0]
DEFAULT_TIME_DIV  = 0.002
DEFAULT_VOLTS_DIV = 0.5
REFRESH_MS   = 30
DATA_BUF_MAX = 100000
GAP_THRESHOLD_MS = 0.3
BATCH_SIZE   = 120

# ============================================================================
#  256-Point 12-bit Sine Look-Up Table (from mcu_control reference)
# ============================================================================
SINE_LUT = np.array([
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
#  DDS Engine — 32-bit phase accumulator with LUT sine + LFSR noise
#  (from function_generator_mcu_control.py reference)
# ============================================================================
class DDSEngine:
    """Per-channel DDS engine: 32-bit phase accumulator, LUT-based sine,
    Galois LFSR for noise. Mathematically equivalent to MCU-side DDS."""

    def __init__(self):
        self.phase_acc = 0           # 32-bit phase accumulator
        self.phase_step = 0          # computed from freq / effective_sample_rate
        self._lfsr = 0x5EEDBEEF      # 32-bit Galois LFSR seed

    def set_freq(self, freq_hz: float, effective_sample_rate: float):
        self.phase_step = int(PHASE_MAX * freq_hz / effective_sample_rate)

    def reset_phase(self):
        self.phase_acc = 0

    def next_sample(self, wave_type: str, amplitude: int) -> int:
        """Compute next 12-bit DAC sample value."""
        self.phase_acc = (self.phase_acc + self.phase_step) & 0xFFFFFFFF
        ph = self.phase_acc
        half = 0x80000000

        if wave_type == "正弦波":
            idx = (ph >> 24) & 0xFF
            val = int(SINE_LUT[idx])
            val = val * amplitude // AMP_MAX
        elif wave_type == "方波":
            val = amplitude if ph < half else 0
        elif wave_type == "三角波":
            if ph < half:
                val = (ph * 2 * amplitude) >> 32
            else:
                val = ((0xFFFFFFFF - ph) * 2 * amplitude) >> 32
        elif wave_type == "锯齿波":
            val = (ph * amplitude) >> 32
        elif wave_type == "直流":
            val = amplitude
        elif wave_type == "噪声":
            # 32-bit Galois LFSR (polynomial: x^32 + x^22 + x^2 + x + 1)
            lsb = self._lfsr & 1
            self._lfsr >>= 1
            if lsb:
                self._lfsr ^= 0x80200003
            val = (self._lfsr & 0xFFF) * amplitude // AMP_MAX
        else:  # "无" / none
            val = 0

        return max(0, min(AMP_MAX, val))


# ============================================================================
#  Multi-Channel TX Thread — shared serial, writes compact frames
#  (takes serial.Serial object; pattern from mcu_control's StreamingThread)
# ============================================================================
class MultiChannelTXThread(QThread):
    """Background DDS + compact frame streaming over shared serial port."""

    stats_updated  = pyqtSignal(int, int)        # total_samples, pps
    error_occurred = pyqtSignal(str)

    def __init__(self, ser: serial.Serial, baudrate: int):
        super().__init__()
        self._ser = ser
        self._baudrate = baudrate
        self._running = False
        self._mutex = QMutex()

        # Per-channel parameters + dedicated DDS engine
        self._ch_params = {}
        for i in range(1, 5):
            self._ch_params[i] = {
                "wave": "无", "freq": 1000.0, "amp": 4095,
                "active": False, "dds": DDSEngine()
            }
        self._sample_count = 0

    def update_channel_params(self, ch_id: int, wave: str, freq: float,
                              amp: int, active: bool):
        with QMutexLocker(self._mutex):
            p = self._ch_params[ch_id]
            p["wave"]   = wave
            p["freq"]   = freq
            p["amp"]    = amp
            p["active"] = active

    def run(self):
        self._running = True
        epoch_start = time.perf_counter()
        t_last_stats = epoch_start
        next_slot = epoch_start
        prev_active_count = 0

        # Local snapshot (double-buffer pattern for lock-free DDS loop)
        local_params = {}
        for i in range(1, 5):
            local_params[i] = {
                "wave": "无", "freq": 1000.0, "amp": 4095,
                "active": False, "dds": DDSEngine()
            }

        # ── Adaptive bandwidth model ──
        max_fps = (self._baudrate / 10.0) / FRAME_LEN
        safe_fps = max_fps * 0.90

        try:
            while self._running:
                # ── Copy shared params to local snapshot ──
                if self._mutex.tryLock():
                    try:
                        for i in range(1, 5):
                            sp = self._ch_params[i]
                            lp = local_params[i]
                            lp["wave"]   = sp["wave"]
                            lp["freq"]   = sp["freq"]
                            lp["amp"]    = sp["amp"]
                            lp["active"] = sp["active"]
                    finally:
                        self._mutex.unlock()

                active_channels = [ch for ch in range(1, 5) if local_params[ch]["active"]]
                active_count = len(active_channels)
                if not active_count:
                    self.msleep(10)
                    continue

                allowed_fps_per_ch = safe_fps / active_count
                transmit_step = math.ceil(SAMPLE_RATE / allowed_fps_per_ch)
                if transmit_step < 1:
                    transmit_step = 1

                actual_fps_per_ch = SAMPLE_RATE / transmit_step
                effective_sample_rate = actual_fps_per_ch * SAMPLES_PER_COMPACT
                frame_interval_s = 1.0 / actual_fps_per_ch

                # ── Adaptive micro-batch sizing ──
                if active_count == 1 and actual_fps_per_ch > 8000:
                    micro_batch = 4
                elif active_count <= 2:
                    micro_batch = 2
                else:
                    micro_batch = 1
                batch_interval_s = micro_batch * frame_interval_s

                if active_count != prev_active_count:
                    # Reset scheduler + DDS phases on channel count change
                    for ch_id in active_channels:
                        lp = local_params[ch_id]
                        lp["dds"].set_freq(lp["freq"], effective_sample_rate)
                        lp["dds"].reset_phase()
                    next_slot = time.perf_counter()
                    prev_active_count = active_count

                # ── Update DDS phase steps (in case freq changed) ──
                for ch_id in active_channels:
                    lp = local_params[ch_id]
                    lp["dds"].set_freq(lp["freq"], effective_sample_rate)

                # ── Precise timing: sleep + spin-wait ──
                wait_until = next_slot
                now = time.perf_counter()
                sleep_s = wait_until - now
                if sleep_s > 0.0:
                    if sleep_s > 0.002:
                        time.sleep(sleep_s - 0.0015)
                    while time.perf_counter() < wait_until:
                        pass

                # ── Generate micro-batch of compact frames ──
                chunk = bytearray()
                for _ in range(micro_batch):
                    for ch_id in active_channels:
                        lp = local_params[ch_id]
                        dds = lp["dds"]
                        wave = lp["wave"]
                        amp  = lp["amp"]

                        s0 = dds.next_sample(wave, amp)
                        s1 = dds.next_sample(wave, amp)
                        s2 = dds.next_sample(wave, amp)

                        a1 = struct.pack('>H', s0 & 0xFFFF)
                        a2 = struct.pack('>H', s1 & 0xFFFF)
                        a3 = struct.pack('>H', s2 & 0xFFFF)

                        ck = (FRAME_H1 + FRAME_H2 + ch_id + CMD_COMPACT +
                              a1[0] + a1[1] + a2[0] + a2[1] + a3[0] + a3[1]) & 0xFF

                        chunk.extend([FRAME_H1, FRAME_H2, ch_id, CMD_COMPACT])
                        chunk.extend(a1)
                        chunk.extend(a2)
                        chunk.extend(a3)
                        chunk.append(ck)
                        self._sample_count += SAMPLES_PER_COMPACT

                if chunk:
                    try:
                        self._ser.write(chunk)
                    except (serial.SerialTimeoutException, serial.SerialException):
                        pass

                next_slot = max(next_slot + batch_interval_s, time.perf_counter())

                # ── Periodic statistics ──
                now = time.perf_counter()
                if now - t_last_stats >= 0.5:
                    elapsed = now - epoch_start
                    pps = int(self._sample_count / elapsed) if elapsed > 0 else 0
                    self.stats_updated.emit(self._sample_count, pps)
                    t_last_stats = now

        except Exception as e:
            self.error_occurred.emit(f"TX stream error: {e}")

    def stop_thread(self):
        self._running = False
        if not self.wait(2000):
            self.terminate()
            self.wait()


# ============================================================================
#  Multi-Channel RX Thread — shared serial, reads + parses frames
#  (takes serial.Serial object; also parses MCU echo frames & status lines)
# ============================================================================
class MultiChannelRXThread(QThread):
    """Background serial RX: FSM parser for CMD=0x01/0x02 data frames,
    CMD=0x03 echo frames, and MCU text status lines (S:...)."""

    samples_ready  = pyqtSignal(int, np.ndarray, np.ndarray)  # ch_id, adc, t_us
    echo_received  = pyqtSignal(int, int, int, int)            # ch_id, s0, s1, s2
    status_updated = pyqtSignal(int, int, int, int)            # ok, bad, fill, lost
    error_occurred = pyqtSignal(str)

    def __init__(self, ser: serial.Serial, baud: int):
        super().__init__()
        self._ser     = ser
        self._baud    = baud
        self._running = False

    def run(self):
        self._running = True
        import math as _math

        # ── Synthetic timestamp engine ──
        max_fps = self._baud / 10.0 / FRAME_LEN
        safe_fps = max_fps * 0.90
        tx_step = _math.ceil(SAMPLE_RATE / safe_fps)
        if tx_step < 1:
            tx_step = 1
        actual_fps = SAMPLE_RATE / tx_step
        effective_rate = actual_fps * 3
        self._dt_ns = int(1_000_000_000 / effective_rate)
        self._synth_t_ns = {ch: 0 for ch in range(1, 5)}

        raw_buf = bytearray()
        adc_batches = {ch: [] for ch in range(1, 5)}
        tms_batches = {ch: [] for ch in range(1, 5)}

        while self._running:
            try:
                ser = self._ser
                if ser is None or not ser.is_open:
                    self.msleep(10)
                    continue

                n = ser.in_waiting
                if n == 0:
                    self.usleep(200)
                    continue

                chunk = ser.read(min(n, 16384))
                raw_buf.extend(chunk)

                ptr = 0
                buf_len = len(raw_buf)
                i = 0  # also used for text line scanning

                # ── First pass: scan for MCU text status lines ──
                while i < buf_len:
                    if raw_buf[i] == ord('S') and i + 1 < buf_len and raw_buf[i + 1] == ord(':'):
                        end = raw_buf.find(b'\n', i)
                        if end < 0:
                            break
                        line = raw_buf[i:end].decode('ascii', errors='ignore')
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

                # ── Second pass: FSM frame parser ──
                while (buf_len - ptr) >= FRAME_LEN:
                    if raw_buf[ptr] != FRAME_H1 or raw_buf[ptr + 1] != FRAME_H2:
                        ptr += 1
                        continue

                    ch_id = raw_buf[ptr + 2]
                    cmd   = raw_buf[ptr + 3]

                    if not (1 <= ch_id <= 4):
                        ptr += 1
                        continue

                    ck_rx   = raw_buf[ptr + 10]
                    ck_calc = sum(raw_buf[ptr : ptr + 10]) & 0xFF
                    if ck_calc != ck_rx:
                        ptr += 1
                        continue

                    if cmd == CMD_COMPACT:
                        # Compact frame: 3 samples with synthetic timestamps
                        a1 = (raw_buf[ptr + 4] << 8) | raw_buf[ptr + 5]
                        a2 = (raw_buf[ptr + 6] << 8) | raw_buf[ptr + 7]
                        a3 = (raw_buf[ptr + 8] << 8) | raw_buf[ptr + 9]

                        for adc_raw in (a1, a2, a3):
                            adc_val = max(0, min(AMP_MAX, adc_raw))
                            synth_ns = self._synth_t_ns.get(ch_id, 0)
                            self._synth_t_ns[ch_id] = synth_ns + self._dt_ns
                            tms_batches[ch_id].append(synth_ns // 1000)
                            adc_batches[ch_id].append(adc_val)

                            if len(adc_batches[ch_id]) >= BATCH_SIZE:
                                self.samples_ready.emit(
                                    ch_id,
                                    np.array(adc_batches[ch_id], dtype=np.uint16),
                                    np.array(tms_batches[ch_id], dtype=np.uint32)
                                )
                                adc_batches[ch_id].clear()
                                tms_batches[ch_id].clear()

                    elif cmd == CMD_DATA:
                        # Standard frame: 1 sample with 4-byte timestamp
                        t_us = ((raw_buf[ptr + 4] << 24) |
                                (raw_buf[ptr + 5] << 16) |
                                (raw_buf[ptr + 6] <<  8) |
                                 raw_buf[ptr + 7]) & 0xFFFFFFFF

                        adc_raw = (raw_buf[ptr + 8] << 8) | raw_buf[ptr + 9]
                        if adc_raw >= 32768:
                            adc_raw -= 65536
                        adc_val = max(0, min(AMP_MAX, adc_raw))

                        adc_batches[ch_id].append(adc_val)
                        tms_batches[ch_id].append(t_us)

                        if len(adc_batches[ch_id]) >= BATCH_SIZE:
                            self.samples_ready.emit(
                                ch_id,
                                np.array(adc_batches[ch_id], dtype=np.uint16),
                                np.array(tms_batches[ch_id], dtype=np.uint32)
                            )
                            adc_batches[ch_id].clear()
                            tms_batches[ch_id].clear()

                    elif cmd == CMD_ECHO:
                        # MCU echo frame: 3 DAC values echoed back
                        s0 = (raw_buf[ptr + 4] << 8) | raw_buf[ptr + 5]
                        s1 = (raw_buf[ptr + 6] << 8) | raw_buf[ptr + 7]
                        s2 = (raw_buf[ptr + 8] << 8) | raw_buf[ptr + 9]
                        self.echo_received.emit(ch_id, s0, s1, s2)

                    else:
                        ptr += 1
                        continue

                    ptr += FRAME_LEN

                if ptr > 0:
                    del raw_buf[:ptr]

            except Exception as e:
                self.error_occurred.emit(f"RX stream error: {e}")
                break

        # Flush remaining batch data
        for ch in range(1, 5):
            if adc_batches[ch]:
                self.samples_ready.emit(
                    ch,
                    np.array(adc_batches[ch], dtype=np.uint16),
                    np.array(tms_batches[ch], dtype=np.uint32)
                )

    def stop_thread(self):
        self._running = False
        if not self.wait(2000):
            self.terminate()
            self.wait()


# ============================================================================
#  Static Waveform Preview Engine (for FG tab's local preview plot)
# ============================================================================
class StaticWaveEngine:
    @staticmethod
    def get_points(wave_type: str, frequency: float, amplitude: int,
                   t_ms: np.ndarray) -> np.ndarray:
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
#  TAB 1 — Function Generator Panel
#  (No serial controls; communicates via main window / shared threads)
# ============================================================================
class FunctionGeneratorPanel(QWidget):
    """4-Channel signal generator with static waveform preview."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._tx_thread: Optional[MultiChannelTXThread] = None

        self.channels = {
            i: {"wave": "无", "freq": 1000, "amp": 4095, "output": False, "curve": None}
            for i in range(1, 5)
        }
        self.current_ch = 1
        self._t_min_current = 0.0
        self._t_max_current = 10.0

        self._init_ui()
        self._load_channel_settings(self.current_ch)

    # ---- Called by main window to inject the shared TX thread ----
    def set_tx_thread(self, thread: MultiChannelTXThread):
        self._tx_thread = thread
        if thread:
            thread.stats_updated.connect(self._on_stream_stats)
            thread.error_occurred.connect(self._on_stream_error)

    def on_serial_disconnected(self):
        """Called when serial is disconnected externally."""
        self._tx_thread = None
        for i in range(1, 5):
            self.channels[i]["output"] = False
            self._out_btns[i].blockSignals(True)
            self._out_btns[i].setChecked(False)
            self._out_btns[i].blockSignals(False)
            self._style_output_btn(i, False)
        self._lbl_stream_stats.setText("串流状态: 链路空闲")

    # ---- UI ----
    def _init_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(12)
        root.addWidget(self._build_control_panel(), stretch=0)
        root.addWidget(self._build_scope_panel(), stretch=1)

    def _build_control_panel(self) -> QWidget:
        p = QWidget()
        p.setFixedWidth(380)
        ly = QVBoxLayout(p)
        ly.setContentsMargins(0, 0, 0, 0)
        ly.setSpacing(10)

        # 1. Channel Selector
        grp_ch = QGroupBox("1. 通道选择 (Channel Select)")
        g_ch = QVBoxLayout(grp_ch)
        self._combo_ch_select = QComboBox()
        for i in range(1, 5):
            self._combo_ch_select.addItem(f"配置通道 CH {i}", i)
        self._combo_ch_select.currentIndexChanged.connect(self._on_current_ch_changed)
        g_ch.addWidget(self._combo_ch_select)
        ly.addWidget(grp_ch)

        # 2. Waveform Selection
        grp_wave = QGroupBox("2. 波形函数选择 (Waveform)")
        g_wave = QVBoxLayout(grp_wave)
        row_w1 = QHBoxLayout()
        self._btn_sine     = self._make_wave_btn("正弦波")
        self._btn_square   = self._make_wave_btn("方波")
        self._btn_triangle = self._make_wave_btn("三角波")
        row_w1.addWidget(self._btn_sine)
        row_w1.addWidget(self._btn_square)
        row_w1.addWidget(self._btn_triangle)

        row_w2 = QHBoxLayout()
        self._btn_sawtooth = self._make_wave_btn("锯齿波")
        self._btn_dc       = self._make_wave_btn("直流")
        self._btn_noise    = self._make_wave_btn("噪声")
        row_w2.addWidget(self._btn_sawtooth)
        row_w2.addWidget(self._btn_dc)
        row_w2.addWidget(self._btn_noise)

        self._wave_group = QButtonGroup(self)
        self._wave_group.setExclusive(False)
        all_btns = [self._btn_sine, self._btn_square, self._btn_triangle,
                    self._btn_sawtooth, self._btn_dc, self._btn_noise]
        for idx, btn in enumerate(all_btns):
            self._wave_group.addButton(btn, idx)
        self._wave_group.buttonClicked.connect(self._on_wave_type_clicked)
        g_wave.addLayout(row_w1)
        g_wave.addLayout(row_w2)
        ly.addWidget(grp_wave)

        # 3. Frequency
        grp_freq = QGroupBox("3. 频率参数微调 (Frequency)")
        g_freq = QVBoxLayout(grp_freq)
        row_f = QHBoxLayout()
        self._lbl_freq_display = QLabel("1 000")
        self._lbl_freq_display.setFont(QFont("Consolas", 22, QFont.Bold))
        self._lbl_freq_display.setStyleSheet(
            "color:#00FF88; background:#151520; padding:2px 8px; border-radius:4px;")
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

        # 4. Amplitude
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

        # 5. Output Matrix (no serial controls here — managed by main window)
        grp_out = QGroupBox("5. 物理输出硬件通道总线矩阵 (Independent Outputs)")
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
        self._lbl_stream_stats = QLabel("串流状态: 请先在上方连接串口")
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
            self.channels[i]["curve"] = self._plot.plot(
                pen=pg.mkPen(CH_COLORS[i], width=2.2))
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

    # ---- Helpers ----
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
        self._sync_to_tx(self.current_ch)
        self._update_plots()

    def _on_freq_slider_moved(self, val: int):
        self.channels[self.current_ch]["freq"] = val
        self._edit_freq.setText(str(val))
        self._lbl_freq_display.setText(f"{val:,}".replace(",", " "))
        self._sync_to_tx(self.current_ch)
        self._update_plots()

    def _on_freq_edited(self, text: str):
        try:
            val = int(text)
            if FREQ_MIN <= val <= FREQ_MAX:
                self.channels[self.current_ch]["freq"] = val
                self._slider_freq.setValue(val)
                self._lbl_freq_display.setText(f"{val:,}".replace(",", " "))
                self._sync_to_tx(self.current_ch)
                self._update_plots()
        except ValueError:
            pass

    def _on_amp_slider_moved(self, val: int):
        self.channels[self.current_ch]["amp"] = val
        self._edit_amp.setText(str(val))
        self._lbl_amp_v.setText(f"({val * VREF / AMP_MAX:.2f} Vpp)")
        self._sync_to_tx(self.current_ch)
        self._update_plots()

    def _on_amp_edited(self, text: str):
        try:
            val = int(text)
            if AMP_MIN <= val <= AMP_MAX:
                self.channels[self.current_ch]["amp"] = val
                self._slider_amp.setValue(val)
                self._lbl_amp_v.setText(f"({val * VREF / AMP_MAX:.2f} Vpp)")
                self._sync_to_tx(self.current_ch)
                self._update_plots()
        except ValueError:
            pass

    def _sync_to_tx(self, ch_id: int):
        if self._tx_thread and self._tx_thread.isRunning():
            ch = self.channels[ch_id]
            self._tx_thread.update_channel_params(
                ch_id, ch["wave"], ch["freq"], ch["amp"], ch["output"])

    def _style_output_btn(self, ch_id: int, checked: bool):
        btn = self._out_btns[ch_id]
        if checked:
            color = CH_COLORS[ch_id]
            btn.setText(f"CH {ch_id} 物理输出: 开启")
            btn.setStyleSheet(
                f"QPushButton {{ background-color:{color}; color:#000; "
                f"border:1px solid white; font-weight:bold; border-radius:4px; }}")
        else:
            btn.setText(f"CH {ch_id} 物理输出: 关闭")
            btn.setStyleSheet(
                "QPushButton { background-color:#2A2A30; color:#666; "
                "border:1px solid #444; border-radius:4px; }")

    def _on_ch_output_toggled(self, ch_id: int, checked: bool):
        self.channels[ch_id]["output"] = checked
        self._style_output_btn(ch_id, checked)

        if self._tx_thread is None or not self._tx_thread.isRunning():
            QMessageBox.warning(self, "串口未连接",
                                "请先在顶部「串口连接」面板中点击连接按钮。")
            btn = self._out_btns[ch_id]
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
            self.channels[ch_id]["output"] = False
            self._style_output_btn(ch_id, False)
            return

        self._sync_to_tx(ch_id)

    def _on_stream_stats(self, total: int, pps: int):
        self._lbl_stream_stats.setText(f"总发送: {total} 样本 | 实时吞吐: {pps} sps")

    def _on_stream_error(self, msg: str):
        self._lbl_stream_stats.setText(f"TX 错误: {msg}")

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
                volt_data = StaticWaveEngine.get_points(
                    ch["wave"], ch["freq"], ch["amp"], t_ms)
                ch["curve"].setData(t_ms, volt_data)

    def cleanup(self):
        pass  # Thread lifecycle managed by main window


# ============================================================================
#  TAB 2 — Oscilloscope Panel
#  (No serial controls; receives data from shared RX thread)
# ============================================================================
class OscilloscopePanel(QWidget):
    """4-Channel digital storage oscilloscope with trigger engine."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self._t_bufs = {ch: deque(maxlen=DATA_BUF_MAX) for ch in range(1, 5)}
        self._v_bufs = {ch: deque(maxlen=DATA_BUF_MAX) for ch in range(1, 5)}
        self._ch_enabled = {1: True, 2: True, 3: False, 4: False}

        self._time_div  = DEFAULT_TIME_DIV
        self._volts_div = DEFAULT_VOLTS_DIV

        self._trig_mode   = "AUTO"
        self._trig_edge   = "RISING"
        self._trig_level  = 1.65
        self._trig_source = 1

        self._t_base = {ch: None for ch in range(1, 5)}

        self._init_ui()

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_scope)
        self._refresh_timer.start(REFRESH_MS)

    # ---- Called by main window to inject the shared RX thread ----
    def set_rx_thread(self, thread: MultiChannelRXThread):
        if thread:
            thread.samples_ready.connect(self._on_samples_received)
            thread.error_occurred.connect(self._on_rx_error)
            thread.echo_received.connect(self._on_echo_received)
            thread.status_updated.connect(self._on_mcu_status)

    def on_serial_connected(self):
        """Reset buffers when a new serial connection is established."""
        for ch in range(1, 5):
            self._t_bufs[ch].clear()
            self._v_bufs[ch].clear()
            self._t_base[ch] = None

    def on_serial_disconnected(self):
        pass

    def _init_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)
        root.addWidget(self._build_ctrl_panel(), stretch=0)
        root.addWidget(self._build_scope_panel(), stretch=1)

    def _build_ctrl_panel(self) -> QWidget:
        p = QWidget()
        p.setMaximumWidth(360)
        ly = QVBoxLayout(p)
        ly.setSpacing(6)

        # 1. Hardware Channels
        grp = QGroupBox("Hardware Channels")
        g = QHBoxLayout(grp)
        self._chk_channels = {}
        for ch in range(1, 5):
            chk = QCheckBox(f"CH{ch}")
            chk.setChecked(self._ch_enabled[ch])
            chk.setStyleSheet(f"QCheckBox {{ color: {CH_COLORS[ch]}; font-weight: bold; }}")
            chk.stateChanged.connect(lambda _, c=ch: self._on_channel_toggled(c))
            g.addWidget(chk)
            self._chk_channels[ch] = chk
        ly.addWidget(grp)

        # 2. Vertical (VOLTS / DIV)
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

        # 3. Horizontal (TIME / DIV)
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
        g.addLayout(row1)
        g.addLayout(row2)
        ly.addWidget(grp)

        # 4. Trigger Engine
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
        self._btn_mode.setStyleSheet(
            "background-color:#0078D4; color:white; font-weight:bold; "
            "border-radius:3px; min-height:24px;")
        row.addWidget(self._btn_mode)
        g.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("Edge:"))
        self._btn_edge = QPushButton("RISING ↗")
        self._btn_edge.setCheckable(True)
        self._btn_edge.clicked.connect(self._on_trig_edge_toggle)
        self._btn_edge.setStyleSheet(
            "background-color:#333; color:#CCC; border:1px solid #555; "
            "border-radius:3px; min-height:24px;")
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

        # 5. System Monitor
        grp = QGroupBox("System Monitor")
        g = QVBoxLayout(grp)
        self._lbl_info = QLabel()
        self._lbl_info.setFont(QFont("Consolas", 9))
        self._lbl_info.setStyleSheet("color:#AAA;")
        g.addWidget(self._lbl_info)
        ly.addWidget(grp)

        # 6. MCU Status (echo feedback)
        grp2 = QGroupBox("MCU Feedback")
        g2 = QVBoxLayout(grp2)
        self._lbl_echo = QLabel("MCU Echo: --")
        self._lbl_echo.setFont(QFont("Consolas", 9))
        self._lbl_echo.setStyleSheet("color:#888;")
        g2.addWidget(self._lbl_echo)
        self._lbl_mcu_status = QLabel("MCU Status: --")
        self._lbl_mcu_status.setFont(QFont("Consolas", 9))
        self._lbl_mcu_status.setStyleSheet("color:#888;")
        g2.addWidget(self._lbl_mcu_status)
        ly.addWidget(grp2)

        ly.addStretch()
        return p

    def _build_scope_panel(self) -> QWidget:
        p = QWidget()
        ly = QVBoxLayout(p)
        ly.setContentsMargins(0, 0, 0, 0)
        self._plot = pg.PlotWidget()
        self._plot.setBackground("#121218")
        self._curves = {}
        for ch in range(1, 5):
            self._curves[ch] = self._plot.plot(
                pen=pg.mkPen(color=CH_COLORS[ch], width=1.6))
            self._curves[ch].setVisible(self._ch_enabled[ch])
        self._trig_line = pg.InfiniteLine(
            angle=0, pen=pg.mkPen(color="#FF3333", style=Qt.DashLine, width=1.5))
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

    # ---- Event Handlers ----
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
        uns  = (base + "background-color:#1E1E2A; color:#777; border:1px solid #333; }"
                " QPushButton:hover { background-color:#2A2A3A; color:#AAA; }")
        for btn in btn_dict.values():
            btn.setStyleSheet(sel if btn.isChecked() else uns)

    def _on_trig_mode_toggle(self, checked: bool):
        if checked:
            self._trig_mode = "NORMAL"
            self._btn_mode.setText("NORMAL (Trig'd)")
            self._btn_mode.setStyleSheet(
                "background-color:#D46400; color:white; font-weight:bold; "
                "border-radius:3px; min-height:24px;")
            self._trig_line.setVisible(True)
        else:
            self._trig_mode = "AUTO"
            self._btn_mode.setText("AUTO (Rolling)")
            self._btn_mode.setStyleSheet(
                "background-color:#0078D4; color:white; font-weight:bold; "
                "border-radius:3px; min-height:24px;")
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

    def _on_samples_received(self, ch_id: int, adc: np.ndarray, tms: np.ndarray):
        if ch_id in self._v_bufs:
            if self._t_base[ch_id] is None and len(tms) > 0:
                self._t_base[ch_id] = int(tms[0])
            base = self._t_base[ch_id] or 0
            self._t_bufs[ch_id].extend(int(t - base) for t in tms)
            self._v_bufs[ch_id].extend(adc)

    def _on_rx_error(self):
        pass  # Error relayed to main window via signal

    def _on_echo_received(self, ch_id: int, s0: int, s1: int, s2: int):
        self._lbl_echo.setText(
            f"MCU Echo CH{ch_id}: [{s0}, {s1}, {s2}] "
            f"({s0*VREF/AMP_MAX:.2f}V, {s1*VREF/AMP_MAX:.2f}V, {s2*VREF/AMP_MAX:.2f}V)")

    def _on_mcu_status(self, ok: int, bad: int, fill: int, lost: int):
        self._lbl_mcu_status.setText(
            f"MCU Status — RX:{ok} Bad:{bad} Buf:{fill} Lost:{lost}")

    def _refresh_scope(self):
        src = self._trig_source
        if len(self._v_bufs[src]) < 50:
            return

        win_ms = self._time_div * H_DIVS * 1000.0

        t_snapshot = {}
        v_snapshot = {}
        for ch in range(1, 5):
            if self._ch_enabled[ch] and len(self._v_bufs[ch]) > 0:
                t_snapshot[ch] = np.array(self._t_bufs[ch], dtype=np.float64) / 1000.0
                v_snapshot[ch] = np.array(self._v_bufs[ch], dtype=np.float64) * VREF / AMP_MAX

        if self._trig_mode == "AUTO":
            latest_ms = -1.0
            for ch in range(1, 5):
                if ch in t_snapshot:
                    t_arr = t_snapshot[ch]
                    v_arr = v_snapshot[ch]
                    gaps = np.diff(t_arr) > GAP_THRESHOLD_MS
                    if np.any(gaps):
                        gap_idx = np.where(gaps)[0] + 1
                        t_arr = np.insert(t_arr, gap_idx, np.nan)
                        v_arr = np.insert(v_arr, gap_idx, np.nan)
                    self._curves[ch].setData(t_arr, v_arr)
                    valid_t = t_arr[~np.isnan(t_arr)]
                    if len(valid_t) > 0 and valid_t[-1] > latest_ms:
                        latest_ms = valid_t[-1]
            if latest_ms > 0:
                self._plot.setXRange(max(0, latest_ms - win_ms), latest_ms, padding=0)

            info_str = "Mode: AUTO (Rolling)\n"
            for ch in range(1, 5):
                if ch in v_snapshot:
                    info_str += (f"CH{ch} Buffer: {len(v_snapshot[ch])} pts | "
                                 f"Max: {np.max(v_snapshot[ch][-200:]):.2f}V\n")
            self._lbl_info.setText(info_str)

        elif self._trig_mode == "NORMAL":
            if src not in t_snapshot:
                self._lbl_info.setText(f"Mode: NORMAL\nWaiting for source CH{src} data...")
                return

            t_src = t_snapshot[src]
            v_src = v_snapshot[src]
            lvl = self._trig_level

            pts_per_ms_est = len(v_src) / (t_src[-1] - t_src[0] + 1e-6)
            post_pts_needed = int(win_ms * 0.9 * pts_per_ms_est)
            pre_pts_needed  = int(win_ms * 0.1 * pts_per_ms_est)

            search_end   = len(v_src) - post_pts_needed
            search_start = max(0, search_end - max(3000, pre_pts_needed + post_pts_needed))
            if search_end <= search_start:
                return

            v_look = v_src[search_start:search_end]
            t_look = t_src[search_start:search_end]

            if self._trig_edge == "RISING":
                condition = (v_look[:-1] < lvl) & (v_look[1:] >= lvl)
            else:
                condition = (v_look[:-1] > lvl) & (v_look[1:] <= lvl)

            trig_indices = np.where(condition)[0]

            if trig_indices.size > 0:
                idx = trig_indices[-1]
                t0, t1 = t_look[idx], t_look[idx + 1]
                v0, v1 = v_look[idx], v_look[idx + 1]
                if abs(v1 - v0) > 1e-6:
                    fraction = (lvl - v0) / (v1 - v0)
                    trig_time = t0 + fraction * (t1 - t0)
                else:
                    trig_time = t1
                trig_abs_idx = search_start + idx + 1

                for ch in range(1, 5):
                    if ch in t_snapshot and ch in v_snapshot:
                        ch_t = t_snapshot[ch]
                        ch_v = v_snapshot[ch]
                        ch_trig_idx = trig_abs_idx + (len(ch_v) - len(v_src))
                        if ch_trig_idx <= 0 or ch_trig_idx >= len(ch_v):
                            continue
                        start_idx = max(0, ch_trig_idx - pre_pts_needed)
                        end_idx   = min(len(ch_v), ch_trig_idx + post_pts_needed)
                        t_seg = ch_t[start_idx:end_idx] - trig_time
                        v_seg = ch_v[start_idx:end_idx]
                        gaps = np.diff(t_seg) > GAP_THRESHOLD_MS
                        if np.any(gaps):
                            gap_idx = np.where(gaps)[0] + 1
                            t_seg = np.insert(t_seg, gap_idx, np.nan)
                            v_seg = np.insert(v_seg, gap_idx, np.nan)
                        self._curves[ch].setData(t_seg, v_seg)

                self._lbl_info.setText(
                    f"Mode: NORMAL (Trig'd on CH{src})\n"
                    f"Trig Level: {lvl:.2f}V | Edge: {self._trig_edge}\n"
                    f"Render Pts: {end_idx - start_idx} | Status: Synchronized"
                )
            else:
                self._lbl_info.setText(
                    f"Mode: NORMAL (Hold/Wait)\nSearching valid edge on CH{src}...")

    @staticmethod
    def _fmt_volts(v: float) -> str:
        return f"{v:.0f}V" if v >= 1 else f"{v:.1f}V"

    @staticmethod
    def _fmt_time(v: float) -> str:
        return f"{v*1000:.0f}ms" if v >= 0.001 else f"{v*1e6:.0f}μs"

    def cleanup(self):
        self._refresh_timer.stop()


# ============================================================================
#  Unified Main Window — Single serial port + Tab Container
# ============================================================================
class UnifiedWorkbench(QMainWindow):
    """Main window: owns the shared serial port, TX thread, RX thread,
    and both Function Generator / Oscilloscope tab panels."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle(
            "统一信号工作台 v2.0 — 信号发生器 + 数字示波器 | Unified Signal Workbench")
        self.setMinimumSize(1280, 800)

        self._serial: Optional[serial.Serial] = None
        self._tx_thread: Optional[MultiChannelTXThread] = None
        self._rx_thread: Optional[MultiChannelRXThread] = None
        self._connected = False

        # ── Central widget: serial bar on top, tabs below ──
        cw = QWidget()
        self.setCentralWidget(cw)
        root = QVBoxLayout(cw)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ── Shared Serial Connection Bar ──
        root.addWidget(self._build_serial_bar())

        # ── Tab Widget ──
        self._tabs = QTabWidget()
        self._tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #2D2D3D;
                background: #101014;
            }
            QTabBar::tab {
                background: #1A1A26;
                color: #888;
                border: 1px solid #333;
                padding: 8px 24px;
                margin-right: 2px;
                font-weight: bold;
                font-size: 13px;
                min-width: 180px;
            }
            QTabBar::tab:selected {
                background: #0078D4;
                color: white;
                border-bottom: 2px solid #00A0FF;
            }
            QTabBar::tab:hover:!selected {
                background: #252535;
                color: #CCC;
            }
        """)

        self._fg_panel = FunctionGeneratorPanel()
        self._dso_panel = OscilloscopePanel()

        self._tabs.addTab(self._fg_panel,  "📡  信号发生器 (Function Generator)")
        self._tabs.addTab(self._dso_panel, "📊  数字示波器 (Digital Oscilloscope)")
        root.addWidget(self._tabs, stretch=1)

        # ── Status Bar ──
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("统一工作台就绪 — 请选择串口并点击连接 | Ready — Select port and Connect")

        # ── Port Scan Timer ──
        self._scan_timer = QTimer(self)
        self._scan_timer.timeout.connect(self._scan_ports)
        self._scan_timer.start(2000)
        self._scan_ports()

    # ========================================================================
    #  Shared Serial Connection Bar (top of main window)
    # ========================================================================
    def _build_serial_bar(self) -> QWidget:
        bar = QWidget()
        bar.setStyleSheet(
            "background-color:#16161E; border:1px solid #2D2D3D; border-radius:6px;")
        ly = QHBoxLayout(bar)
        ly.setContentsMargins(12, 6, 12, 6)
        ly.setSpacing(12)

        # Port
        ly.addWidget(QLabel("串口 (Port):"))
        self._combo_port = QComboBox()
        self._combo_port.setMinimumWidth(140)
        ly.addWidget(self._combo_port)

        # Baud
        ly.addWidget(QLabel("波特率 (Baud):"))
        self._combo_baud = QComboBox()
        self._combo_baud.addItems(["115200", "230400", "460800", "921600", "1000000", "2000000"])
        self._combo_baud.setCurrentText("2000000")
        ly.addWidget(self._combo_baud)

        # Connect / Disconnect button
        self._btn_connect = QPushButton("🔌 连接 (Connect)")
        self._btn_connect.setMinimumHeight(34)
        self._btn_connect.setMinimumWidth(150)
        self._btn_connect.clicked.connect(self._on_connect_clicked)
        self._style_connect_btn()
        ly.addWidget(self._btn_connect)

        # Connection status indicator
        self._lbl_conn_status = QLabel("⚪ 未连接")
        self._lbl_conn_status.setFont(QFont("Consolas", 10))
        self._lbl_conn_status.setStyleSheet("color:#888; padding:4px 8px;")
        ly.addWidget(self._lbl_conn_status)

        ly.addStretch()
        return bar

    def _style_connect_btn(self):
        if self._connected:
            self._btn_connect.setText("🔌 断开 (Disconnect)")
            self._btn_connect.setStyleSheet(
                "QPushButton { background-color:#D13438; color:white; font-weight:bold; "
                "border-radius:4px; font-size:13px; min-height:34px; } "
                "QPushButton:hover { background-color:#A4262C; }")
        else:
            self._btn_connect.setText("🔌 连接 (Connect)")
            self._btn_connect.setStyleSheet(
                "QPushButton { background-color:#0078D4; color:white; font-weight:bold; "
                "border-radius:4px; font-size:13px; min-height:34px; } "
                "QPushButton:hover { background-color:#106EBE; }")

    # ========================================================================
    #  Serial Connection Management (single shared serial port)
    # ========================================================================
    def _on_connect_clicked(self):
        if self._connected:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self):
        port = self._combo_port.currentText()
        if not port or port.startswith("("):
            QMessageBox.warning(self, "错误", "请选择有效的串口。")
            return

        try:
            baud = int(self._combo_baud.currentText())
        except ValueError:
            baud = 2000000

        # ── Open the single shared serial port ──
        try:
            self._serial = serial.Serial(
                port=port, baudrate=baud, timeout=0.01, write_timeout=0.01)
            self._serial.set_buffer_size(rx_size=1024 * 1024)
        except Exception as e:
            QMessageBox.critical(self, "串口错误", f"无法打开 {port}: {e}")
            return

        self._connected = True
        self._combo_port.setEnabled(False)
        self._combo_baud.setEnabled(False)
        self._style_connect_btn()
        self._lbl_conn_status.setText(f"🟢 已连接 {port} @ {baud} bps")

        # ── Start RX thread (always running when connected) ──
        self._rx_thread = MultiChannelRXThread(self._serial, baud)
        self._rx_thread.error_occurred.connect(self._on_rx_error)
        self._rx_thread.start()
        self._dso_panel.set_rx_thread(self._rx_thread)
        self._dso_panel.on_serial_connected()

        # ── Start TX thread (runs continuously; idle when no active channels) ──
        self._tx_thread = MultiChannelTXThread(self._serial, baud)
        self._tx_thread.error_occurred.connect(self._on_tx_error)
        self._tx_thread.start()
        self._fg_panel.set_tx_thread(self._tx_thread)

        self._status_bar.showMessage(f"已连接 {port} @ {baud} bps — TX+RX 双工运行中")

    def _disconnect_serial(self):
        # ── Stop TX thread ──
        if self._tx_thread:
            self._tx_thread.stop_thread()
            self._tx_thread = None
        self._fg_panel.on_serial_disconnected()

        # ── Stop RX thread ──
        if self._rx_thread:
            self._rx_thread.stop_thread()
            self._rx_thread = None
        self._dso_panel.on_serial_disconnected()

        # ── Close serial port ──
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

        self._connected = False
        self._combo_port.setEnabled(True)
        self._combo_baud.setEnabled(True)
        self._style_connect_btn()
        self._lbl_conn_status.setText("⚪ 未连接")
        self._status_bar.showMessage("已断开 — 串口释放")

    def _on_tx_error(self, msg: str):
        self._status_bar.showMessage(f"TX 错误: {msg}")

    def _on_rx_error(self, msg: str):
        self._status_bar.showMessage(f"RX 错误: {msg}")
        # Auto-disconnect on RX error
        if self._connected:
            self._disconnect_serial()

    # ========================================================================
    #  Port Scanning
    # ========================================================================
    def _scan_ports(self):
        if self._connected:
            return
        cur = self._combo_port.currentText()
        ports = serial.tools.list_ports.comports()
        names = sorted(set(p.device for p in ports))
        self._combo_port.blockSignals(True)
        self._combo_port.clear()
        if names:
            self._combo_port.addItems(names)
            if cur in names:
                self._combo_port.setCurrentText(cur)
        else:
            self._combo_port.addItem("(未检测到可用端口)")
        self._combo_port.blockSignals(False)

    # ========================================================================
    #  Tab Change
    # ========================================================================
    def _on_tab_changed(self, index: int):
        names = ["信号发生器", "数字示波器"]
        if 0 <= index < len(names):
            self._status_bar.showMessage(
                f"当前页面: {names[index]} | Current Page: {names[index]}")

    # ========================================================================
    #  Close Event
    # ========================================================================
    def closeEvent(self, event):
        self._scan_timer.stop()
        self._fg_panel.cleanup()
        self._dso_panel.cleanup()
        if self._connected:
            self._disconnect_serial()
        event.accept()


# ============================================================================
#  Application Entry Point
# ============================================================================
def main():
    try:
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    except AttributeError:
        pass

    app = QApplication(sys.argv)
    font = QFont("Microsoft YaHei")
    font.setPointSize(9)
    app.setFont(font)

    app.setStyleSheet("""
        QMainWindow { background-color:#101014; }
        QGroupBox { font-weight:bold; border:1px solid #2D2D3D; border-radius:6px;
                    margin-top:10px; padding-top:14px; color:#A5A5B5; }
        QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 3px; color:#008BE3; }
        QLabel { color:#B0B0BD; }
        QComboBox { background:#1A1A26; color:#DDD; border:1px solid #3A3A4A;
                    padding:3px 6px; border-radius:4px; }
        QComboBox QAbstractItemView { background:#1A1A26; color:#DDD;
                    selection-background-color:#0078D4; }
        QLineEdit { background:#1A1A26; color:#FFF; border:1px solid #3A3A4A;
                    padding:4px 6px; border-radius:4px; font-family:Consolas; }
        QSlider::groove:horizontal { height:5px; background:#252535; border-radius:2px; }
        QSlider::handle:horizontal { background:#0078D4; border:1px solid #005A9E;
                    width:16px; margin:-5px 0; border-radius:8px; }
        QPushButton { background-color:#2A2A35; color:#AAA; border:1px solid #444;
                    border-radius:4px; min-height:30px; }
        QPushButton:hover { background-color:#353545; color:#FFF; }
        QPushButton:checked { background-color:#0078D4; color:white; font-weight:bold; }
        QStatusBar { color:#777; background:#0A0A0F; border-top:1px solid #222; }
        QCheckBox { color:#B0B0BD; }
        QCheckBox::indicator { width:16px; height:16px; }
    """)

    win = UnifiedWorkbench()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
