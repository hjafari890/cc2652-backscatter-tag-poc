"""
backscatter_poc_dashboard.py
From-scratch unified LaunchPad CC2652R1 MCU + NI USRP-2900 PoC Dashboard.

Uses native uhd.usrp.MultiUSRP + PyQt5 + pyqtgraph cleanly standalone.
"""

from __future__ import annotations

import argparse
import ctypes
from datetime import datetime
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Deque, Dict, List, Optional
from collections import deque

import numpy as np
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from poc_sdr_engine import (
    CENTER_FREQ_HZ,
    HAS_UHD,
    LSB_OFFSET_HZ,
    PocPacketCodec,
    LoopbackSignalState,
    SdrTelemetryFrame,
    TX_CW_OFFSET_HZ,
    UHD_IMPORT_ERROR,
    USB_OFFSET_HZ,
    UsrpWorker,
    Win32Serial,
    _COMMTIMEOUTS,
    _DCB,
    check_xds110_usb,
    compute_spectrum_and_snr,
    list_serial_ports,
)

DASHBOARD_DIR = Path(__file__).resolve().parent
POC_ROOT = DASHBOARD_DIR.parent
FIRMWARE_DIR = POC_ROOT / "firmware"
BUILD_SCRIPT = FIRMWARE_DIR / "build_and_flash.ps1"


class FlashWorker(QThread):
    """Runs build_and_flash.ps1 asynchronously and streams logs to the MCU console."""

    log_line = pyqtSignal(str)
    finished_status = pyqtSignal(int, str)  # (exit_code, action_label)

    def __init__(self, action_label: str, ps_args: List[str], parent=None) -> None:
        super().__init__(parent)
        self.action_label = action_label
        self.ps_args = ps_args

    def run(self) -> None:
        cmd = [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(BUILD_SCRIPT),
        ] + self.ps_args
        self.log_line.emit(f"$ {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            assert proc.stdout is not None
            for line in proc.stdout:
                cleaned = line.rstrip("\r\n")
                if cleaned:
                    self.log_line.emit(cleaned)
            rc = proc.wait()
            self.finished_status.emit(int(rc), self.action_label)
        except Exception as exc:
            self.log_line.emit(f"[ERROR] Flash runner failed: {exc}")
            self.finished_status.emit(1, self.action_label)


def _make_badge(text: str, bg: str, fg: str = "#f8fafc") -> QLabel:
    lbl = QLabel(text)
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setStyleSheet(
        f"background-color: {bg}; color: {fg}; padding: 4px 10px; "
        "border-radius: 5px; font-weight: 600; font-size: 12px;"
    )
    return lbl


def _set_badge(lbl: QLabel, text: str, bg: str, fg: str = "#f8fafc") -> None:
    lbl.setText(text)
    lbl.setStyleSheet(
        f"background-color: {bg}; color: {fg}; padding: 4px 10px; "
        "border-radius: 5px; font-weight: 600; font-size: 12px;"
    )


class BackscatterPocWindow(QMainWindow):
    """Unified PoC window combining MCU flashing/UART control and USRP-2900 SDR DSP."""

    def __init__(self, auto_start_hw: bool = True, initial_loopback: bool = False) -> None:
        super().__init__()
        self.setWindowTitle("2.4 GHz Backscatter PoC Testbed — CC2652R1 + NI USRP-2900")
        self.resize(1460, 920)

        self.usrp_worker: Optional[UsrpWorker] = None
        self.flash_worker: Optional[FlashWorker] = None
        self.serial_conn: Optional[Win32Serial] = None
        self._uart_rx_buf = bytearray()
        self._reconnect_com_after_flash: Optional[str] = None

        self.snr_times: Deque[float] = deque(maxlen=600)  # ~30 s @ 20 Hz
        self.snr_values: Deque[float] = deque(maxlen=600)
        self.phase_delta_values: Deque[float] = deque(maxlen=600)
        self._phase_ref_deg: Optional[float] = None
        self._last_carrier_phase_deg: float = 0.0
        self._toggle_state_50ohm = False
        self._t0 = time.monotonic()

        self._build_ui()
        self._apply_dark_palette()
        self.refresh_probes_and_ports()

        self.uart_timer = QTimer(self)
        self.uart_timer.setInterval(100)
        self.uart_timer.timeout.connect(self._poll_uart)
        self.uart_timer.start()

        self.load_toggle_timer = QTimer(self)
        self.load_toggle_timer.setInterval(500)
        self.load_toggle_timer.timeout.connect(self._on_load_toggle_tick)

        if initial_loopback:
            self.chk_loopback.blockSignals(True)
            self.chk_loopback.setChecked(True)
            self.chk_loopback.blockSignals(False)
            self.btn_sim_btn2.setEnabled(True)
            self.start_usrp_worker()
        elif auto_start_hw:
            QTimer.singleShot(150, self.start_usrp_worker)

    def _apply_dark_palette(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background-color: #0f172a;
                color: #e2e8f0;
                font-family: "Segoe UI", sans-serif;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #334155;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: 600;
                color: #93c5fd;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #1e293b;
                border: 1px solid #475569;
                border-radius: 5px;
                padding: 5px 10px;
                color: #f1f5f9;
            }
            QPushButton:hover {
                background-color: #334155;
                border-color: #60a5fa;
            }
            QPushButton:pressed {
                background-color: #0284c7;
            }
            QPushButton:disabled {
                color: #64748b;
                border-color: #1e293b;
            }
            QComboBox, QLineEdit, QDoubleSpinBox, QPlainTextEdit, QTableWidget {
                background-color: #020617;
                border: 1px solid #334155;
                border-radius: 4px;
                color: #e2e8f0;
                padding: 3px 6px;
            }
            QHeaderView::section {
                background-color: #1e293b;
                color: #93c5fd;
                padding: 4px;
                border: none;
                border-right: 1px solid #334155;
                font-weight: 600;
            }
            """
        )

    def _build_ui(self) -> None:
        # Import pyqtgraph only after QApplication exists (required on Windows radioconda)
        import pyqtgraph as pg

        pg.setConfigOptions(antialias=False, useOpenGL=False)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(6)

        # ---- Top Status Header ----
        status_row = QHBoxLayout()
        self.badge_usrp = _make_badge("USRP-2900: Initializing...", "#475569")
        self.badge_xds = _make_badge("XDS110: Checking...", "#475569")
        self.badge_uart = _make_badge("UART: Disconnected", "#475569")
        self.badge_snr = _make_badge("Sideband SNR: -- dB", "#1e293b")
        self.badge_pkts = _make_badge("CRC OK: 0 | Fail: 0", "#1e293b")
        self.badge_clip = _make_badge("ADC: OK", "#14532d")

        for w in (
            self.badge_usrp,
            self.badge_xds,
            self.badge_uart,
            self.badge_snr,
            self.badge_pkts,
            self.badge_clip,
        ):
            status_row.addWidget(w)
        root_layout.addLayout(status_row)

        # ---- Main Horizontal Split: Sidebar Controls + Plots/Tables ----
        main_split = QSplitter(Qt.Horizontal)
        root_layout.addWidget(main_split, stretch=1)

        sidebar = QWidget()
        sidebar.setMinimumWidth(340)
        sidebar.setMaximumWidth(400)
        side_layout = QVBoxLayout(sidebar)
        side_layout.setContentsMargins(0, 0, 4, 0)
        side_layout.setSpacing(8)

        # --- USRP-2900 Control Box ---
        grp_usrp = QGroupBox("NI USRP-2900 RF & Stream")
        g_usrp = QGridLayout(grp_usrp)

        g_usrp.addWidget(QLabel("Center Freq (MHz):"), 0, 0)
        self.spin_freq = QDoubleSpinBox()
        self.spin_freq.setRange(2400.0, 2483.5)
        self.spin_freq.setDecimals(3)
        self.spin_freq.setSingleStep(0.5)
        self.spin_freq.setValue(CENTER_FREQ_HZ / 1e6)
        self.spin_freq.valueChanged.connect(self._on_rf_changed)
        g_usrp.addWidget(self.spin_freq, 0, 1, 1, 2)

        self.chk_tx_cw = QCheckBox("Transmit +20 kHz CW Carrier (TX/RX)")
        self.chk_tx_cw.setChecked(True)
        self.chk_tx_cw.toggled.connect(self._on_rf_changed)
        g_usrp.addWidget(self.chk_tx_cw, 1, 0, 1, 3)

        g_usrp.addWidget(QLabel("TX Gain (dB):"), 2, 0)
        self.slider_tx = QSlider(Qt.Horizontal)
        self.slider_tx.setRange(0, 89)
        self.slider_tx.setValue(68)
        self.slider_tx.valueChanged.connect(self._on_rf_changed)
        self.lbl_tx_val = QLabel("68 dB")
        self.lbl_tx_val.setMinimumWidth(46)
        g_usrp.addWidget(self.slider_tx, 2, 1)
        g_usrp.addWidget(self.lbl_tx_val, 2, 2)

        g_usrp.addWidget(QLabel("RX Gain (dB):"), 3, 0)
        self.slider_rx = QSlider(Qt.Horizontal)
        self.slider_rx.setRange(0, 76)
        self.slider_rx.setValue(45)
        self.slider_rx.valueChanged.connect(self._on_rf_changed)
        self.lbl_rx_val = QLabel("45 dB")
        self.lbl_rx_val.setMinimumWidth(46)
        g_usrp.addWidget(self.slider_rx, 3, 1)
        g_usrp.addWidget(self.lbl_rx_val, 3, 2)

        self.btn_usrp_connect = QPushButton("Reconnect USRP")
        self.btn_usrp_connect.clicked.connect(self.start_usrp_worker)
        self.btn_usrp_stop = QPushButton("Stop Stream")
        self.btn_usrp_stop.clicked.connect(self.stop_usrp_worker)
        self.btn_autotune = QPushButton("Auto-Tune Gains")
        self.btn_autotune.clicked.connect(self._on_autotune)
        g_usrp.addWidget(self.btn_usrp_connect, 4, 0)
        g_usrp.addWidget(self.btn_usrp_stop, 4, 1)
        g_usrp.addWidget(self.btn_autotune, 4, 2)

        self.btn_zero_phase = QPushButton("Zero Phase Ref")
        self.btn_zero_phase.clicked.connect(self._on_zero_phase)
        self.btn_load_toggle = QPushButton("1 Hz Load Toggle (OPEN <-> LOAD50)")
        self.btn_load_toggle.setCheckable(True)
        self.btn_load_toggle.toggled.connect(self._on_load_toggle_btn)
        g_usrp.addWidget(self.btn_zero_phase, 5, 0)
        g_usrp.addWidget(self.btn_load_toggle, 5, 1, 1, 2)

        self.chk_loopback = QCheckBox("Offline Loopback Test Source (no RF HW)")
        self.chk_loopback.setChecked(False)
        self.chk_loopback.toggled.connect(self._on_loopback_toggled)
        g_usrp.addWidget(self.chk_loopback, 6, 0, 1, 3)

        self.lbl_rf_metrics = QLabel("Carrier(+20k): -- dBFS (Δ0.0°) | Noise: -- dBFS")
        self.lbl_rf_metrics.setStyleSheet("color: #94a3b8; font-size: 11px;")
        g_usrp.addWidget(self.lbl_rf_metrics, 7, 0, 1, 3)

        side_layout.addWidget(grp_usrp)

        # --- LaunchPad CC2652R1 Control Box ---
        grp_mcu = QGroupBox("LAUNCHXL-CC26X2R1 Direct DIO21 Control (No AND Gate)")
        g_mcu = QGridLayout(grp_mcu)

        self.btn_flash_pkt = QPushButton("Build & Flash Packets")
        self.btn_flash_pkt.clicked.connect(lambda: self._run_flash("Flash Pkt", ["-Stage", "Pkt"]))
        self.btn_flash_cw = QPushButton("Build & Flash 200kHz CW")
        self.btn_flash_cw.clicked.connect(lambda: self._run_flash("Flash CW", ["-Stage", "CW"]))
        self.btn_erase = QPushButton("Erase MCU Flash")
        self.btn_erase.clicked.connect(lambda: self._run_flash("Erase Flash", ["-EraseOnly"]))
        g_mcu.addWidget(self.btn_flash_pkt, 0, 0)
        g_mcu.addWidget(self.btn_flash_cw, 0, 1)
        g_mcu.addWidget(self.btn_erase, 1, 0, 1, 2)

        g_mcu.addWidget(QLabel("UART Port:"), 2, 0)
        self.cmb_port = QComboBox()
        g_mcu.addWidget(self.cmb_port, 2, 1)

        uart_btn_row = QHBoxLayout()
        self.btn_refresh_usb = QPushButton("Scan USB/COM")
        self.btn_refresh_usb.clicked.connect(self.refresh_probes_and_ports)
        self.btn_uart_toggle = QPushButton("Connect UART")
        self.btn_uart_toggle.clicked.connect(self._toggle_uart)
        uart_btn_row.addWidget(self.btn_refresh_usb)
        uart_btn_row.addWidget(self.btn_uart_toggle)
        g_mcu.addLayout(uart_btn_row, 3, 0, 1, 2)

        mode_row = QHBoxLayout()
        mode_buttons = (
            ("BTN-1: Load 50Ω (DC)", "LOAD50"),
            ("BTN-1: 200kHz CW", "CW"),
            ("BTN-1: Load Open (0V)", "OPEN"),
            ("BTN-2: Count Pkts", "PKT"),
        )
        for label, cmd_mode in mode_buttons:
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, m=cmd_mode: self._send_mcu_mode(m))
            mode_row.addWidget(b)
        g_mcu.addLayout(mode_row, 4, 0, 1, 2)

        pay_row = QHBoxLayout()
        pay_row.addWidget(QLabel("ASCII (2B):"))
        self.edit_payload = QLineEdit("OK")
        self.edit_payload.setMaxLength(2)
        self.edit_payload.setMaximumWidth(60)
        self.btn_send_pay = QPushButton("Set Payload")
        self.btn_send_pay.clicked.connect(self._send_mcu_payload)
        self.btn_sim_btn2 = QPushButton("Sim BTN-2 (Loopback)")
        self.btn_sim_btn2.setEnabled(False)
        self.btn_sim_btn2.clicked.connect(self._on_sim_btn2)
        pay_row.addWidget(self.edit_payload)
        pay_row.addWidget(self.btn_send_pay)
        pay_row.addWidget(self.btn_sim_btn2)
        g_mcu.addLayout(pay_row, 5, 0, 1, 2)

        self.lbl_mcu_state = QLabel("MCU Telemetry: Waiting for UART / standalone BTN-1/BTN-2...")
        self.lbl_mcu_state.setWordWrap(True)
        self.lbl_mcu_state.setStyleSheet("color: #94a3b8; font-size: 11px;")
        g_mcu.addWidget(self.lbl_mcu_state, 6, 0, 1, 2)

        side_layout.addWidget(grp_mcu)

        # --- MCU Console / Flash Output ---
        grp_con = QGroupBox("MCU UART & Flash Console")
        v_con = QVBoxLayout(grp_con)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(300)
        self.console.setFont(QFont("Consolas", 9))
        v_con.addWidget(self.console)
        side_layout.addWidget(grp_con, stretch=1)

        main_split.addWidget(sidebar)

        # ---- Right Workspace: 3 Plots + Decoded Packet Table ----
        right_split = QSplitter(Qt.Vertical)
        main_split.addWidget(right_split)
        main_split.setStretchFactor(1, 1)

        plots_widget = QWidget()
        plots_layout = QVBoxLayout(plots_widget)
        plots_layout.setContentsMargins(4, 0, 0, 0)
        plots_layout.setSpacing(6)

        # Plot 1: Spectrum
        self.plot_spec = pg.PlotWidget(
            title="RF Power Spectrum (+20 kHz CW Carrier, -180/+220 kHz Sidebands)"
        )
        self.plot_spec.setBackground("#020617")
        self.plot_spec.showGrid(x=True, y=True, alpha=0.25)
        self.plot_spec.setLabel("bottom", "Frequency Offset", units="kHz")
        self.plot_spec.setLabel("left", "Magnitude", units="dBFS")
        self.plot_spec.setYRange(-115, 5)
        self.plot_spec.setXRange(-500, 500)
        self.curve_spec = self.plot_spec.plot(pen=pg.mkPen("#38bdf8", width=1.4))
        for x_ref, col in (
            (TX_CW_OFFSET_HZ / 1e3, "#f59e0b"),
            (LSB_OFFSET_HZ / 1e3, "#22c55e"),
            (USB_OFFSET_HZ / 1e3, "#22c55e"),
        ):
            line = pg.InfiniteLine(pos=x_ref, angle=90, pen=pg.mkPen(col, style=Qt.DashLine, width=1.0))
            self.plot_spec.addItem(line)
        plots_layout.addWidget(self.plot_spec, stretch=2)

        # Bottom Plots Row: Envelope Oscilloscope + Dual SNR / Phase Strip Chart
        sub_plots = QHBoxLayout()

        self.plot_env = pg.PlotWidget(title="-180/+220 kHz OOK Matched Envelope (1.0 kbps, 200 ms window)")
        self.plot_env.setBackground("#020617")
        self.plot_env.showGrid(x=True, y=True, alpha=0.25)
        self.plot_env.setLabel("bottom", "Relative Time", units="ms")
        self.plot_env.setLabel("left", "Amplitude")
        self.curve_env = self.plot_env.plot(pen=pg.mkPen("#4ade80", width=1.5))
        sub_plots.addWidget(self.plot_env, stretch=2)

        self.plot_snr = pg.PlotWidget(title="Sideband SNR (dB, pink) & Carrier ΔPhase (°/10, cyan) — 30 s")
        self.plot_snr.setBackground("#020617")
        self.plot_snr.showGrid(x=True, y=True, alpha=0.25)
        self.plot_snr.addLegend(offset=(10, 10))
        self.plot_snr.setLabel("bottom", "Elapsed Time", units="s")
        self.plot_snr.setLabel("left", "SNR (dB) | ΔPhase (°/10)")
        self.curve_snr = self.plot_snr.plot(name="SNR (dB)", pen=pg.mkPen("#f472b6", width=1.8))
        self.curve_phase = self.plot_snr.plot(name="ΔPhase/10", pen=pg.mkPen("#22d3ee", width=1.5))
        sub_plots.addWidget(self.plot_snr, stretch=1)

        plots_layout.addLayout(sub_plots, stretch=2)
        right_split.addWidget(plots_widget)

        # Decoded Packet Table
        tbl_container = QWidget()
        tbl_layout = QVBoxLayout(tbl_container)
        tbl_layout.setContentsMargins(4, 2, 0, 0)
        tbl_hdr = QHBoxLayout()
        tbl_hdr.addWidget(QLabel("Decoded Backscatter PoC Packets (88-bit @ 1.0 kbps, CRC-8-CCITT)"))
        self.btn_clear_pkts = QPushButton("Clear Table")
        self.btn_clear_pkts.clicked.connect(self._clear_packets)
        tbl_hdr.addStretch(1)
        tbl_hdr.addWidget(self.btn_clear_pkts)
        tbl_layout.addLayout(tbl_hdr)

        self.pkt_table = QTableWidget(0, 8)
        self.pkt_table.setHorizontalHeaderLabels(
            ["Time", "Seq", "BTN-2", "Pay Hex", "ASCII", "CRC-8", "SNR (dB)", "Frame Bytes (Hex)"]
        )
        self.pkt_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.pkt_table.horizontalHeader().setStretchLastSection(True)
        self.pkt_table.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl_layout.addWidget(self.pkt_table)

        right_split.addWidget(tbl_container)
        right_split.setSizes([580, 260])

    # ---- Logging & Hardware Discovery ----
    def append_log(self, msg: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.console.appendPlainText(f"[{stamp}] {msg}")

    def refresh_probes_and_ports(self) -> None:
        info = check_xds110_usb()
        present = bool(info["present"])
        ports: List = list(info["ports"])  # type: ignore[arg-type]

        if present:
            _set_badge(self.badge_xds, "XDS110: Connected", "#14532d")
        else:
            _set_badge(self.badge_xds, "XDS110: Not Detected", "#7f1d1d")
        self.append_log(str(info["summary"]))

        current = self.cmb_port.currentData()
        self.cmb_port.clear()
        for com, reg_name in ports:
            self.cmb_port.addItem(f"{com} ({reg_name})", com)
        if current:
            idx = self.cmb_port.findData(current)
            if idx >= 0:
                self.cmb_port.setCurrentIndex(idx)
        elif self.cmb_port.count() > 0:
            self.cmb_port.setCurrentIndex(0)

        if not ports:
            self.append_log("No COM ports found in Windows SERIALCOMM (LaunchPad USB data/J101 check).")

    # ---- USRP Worker Lifecycle ----
    def start_usrp_worker(self) -> None:
        self.stop_usrp_worker()
        self._phase_ref_deg = None

        freq_hz = self.spin_freq.value() * 1e6
        tx_g = float(self.slider_tx.value())
        rx_g = float(self.slider_rx.value())
        tx_cw = self.chk_tx_cw.isChecked()
        loopback = self.chk_loopback.isChecked()

        self.usrp_worker = UsrpWorker(
            center_freq_hz=freq_hz,
            tx_gain_db=tx_g,
            rx_gain_db=rx_g,
            tx_cw_enabled=tx_cw,
            loopback_test=loopback,
            parent=self,
        )
        self.usrp_worker.telemetry_ready.connect(self._on_telemetry)
        self.usrp_worker.status_changed.connect(self._on_usrp_status)
        self.usrp_worker.error_occurred.connect(self._on_usrp_error)
        self.usrp_worker.autotune_finished.connect(self._on_autotune_done)
        self.usrp_worker.start()

    def stop_usrp_worker(self) -> None:
        if self.usrp_worker is not None:
            self.usrp_worker.stop_worker(3000)
            self.usrp_worker.deleteLater()
            self.usrp_worker = None

    def _on_loopback_toggled(self, checked: bool) -> None:
        self.btn_sim_btn2.setEnabled(checked)
        self.start_usrp_worker()

    def _on_rf_changed(self) -> None:
        tx_val = self.slider_tx.value()
        rx_val = self.slider_rx.value()
        self.lbl_tx_val.setText(f"{tx_val} dB")
        self.lbl_rx_val.setText(f"{rx_val} dB")
        if self.usrp_worker is not None:
            self.usrp_worker.set_rf_params(
                center_freq_hz=self.spin_freq.value() * 1e6,
                tx_gain_db=float(tx_val),
                rx_gain_db=float(rx_val),
                tx_cw_enabled=self.chk_tx_cw.isChecked(),
            )

    def _on_autotune(self) -> None:
        if self.usrp_worker is None or not self.usrp_worker.isRunning():
            self.append_log("Auto-Tune skipped: USRP worker is not running.")
            return
        self.append_log("Running Auto-Tune sweep across TX/RX gain pairs...")
        self.usrp_worker.request_auto_tune()

    def _on_autotune_done(self, tx_g: float, rx_g: float, best_snr: float) -> None:
        self.slider_tx.blockSignals(True)
        self.slider_rx.blockSignals(True)
        self.slider_tx.setValue(int(round(tx_g)))
        self.slider_rx.setValue(int(round(rx_g)))
        self.lbl_tx_val.setText(f"{int(round(tx_g))} dB")
        self.lbl_rx_val.setText(f"{int(round(rx_g))} dB")
        self.slider_tx.blockSignals(False)
        self.slider_rx.blockSignals(False)
        self.append_log(f"Auto-Tune selected TX={tx_g:.0f} dB, RX={rx_g:.0f} dB (SNR={best_snr:.1f} dB)")

    def _on_usrp_status(self, message: str, connected: bool) -> None:
        if self.chk_loopback.isChecked() and connected:
            _set_badge(self.badge_usrp, "SDR: Offline Loopback Test", "#854d0e")
        elif connected:
            _set_badge(self.badge_usrp, f"USRP-2900: {message}", "#14532d")
        else:
            _set_badge(self.badge_usrp, f"USRP-2900: {message}", "#7f1d1d")

    def _on_usrp_error(self, err: str) -> None:
        self.append_log(f"[SDR] {err}")

    def _on_zero_phase(self) -> None:
        self._phase_ref_deg = self._last_carrier_phase_deg
        self.append_log(f"Zeroed +20 kHz carrier phase reference @ {self._phase_ref_deg:+.1f} deg")

    def _on_load_toggle_btn(self, checked: bool) -> None:
        if checked:
            self._toggle_state_50ohm = False
            self.load_toggle_timer.start()
            self.append_log("Started 1 Hz load toggle test (OPEN <-> LOAD50)")
        else:
            self.load_toggle_timer.stop()
            self.append_log("Stopped 1 Hz load toggle test")

    def _on_load_toggle_tick(self) -> None:
        self._toggle_state_50ohm = not self._toggle_state_50ohm
        mode_name = "LOAD50" if self._toggle_state_50ohm else "OPEN"
        self._send_mcu_mode(mode_name)

    def _on_telemetry(self, frame: SdrTelemetryFrame) -> None:
        self.curve_spec.setData(frame.freqs_khz, frame.spectrum_db)
        self.curve_env.setData(frame.env_ms, frame.env_mag)

        self._last_carrier_phase_deg = frame.carrier_phase_deg
        if self._phase_ref_deg is None:
            self._phase_ref_deg = frame.carrier_phase_deg
        delta_phase = ((frame.carrier_phase_deg - self._phase_ref_deg + 180.0) % 360.0) - 180.0

        elapsed = time.monotonic() - self._t0
        self.snr_times.append(elapsed)
        self.snr_values.append(frame.sideband_snr_db)
        self.phase_delta_values.append(delta_phase / 10.0)
        t_list = list(self.snr_times)
        self.curve_snr.setData(t_list, list(self.snr_values))
        self.curve_phase.setData(t_list, list(self.phase_delta_values))

        _set_badge(
            self.badge_snr,
            f"Sideband SNR: {frame.sideband_snr_db:+.1f} dB",
            "#14532d" if frame.sideband_snr_db >= 6.0 else "#1e293b",
        )
        _set_badge(
            self.badge_pkts,
            f"CRC OK: {frame.total_crc_ok} | Fail: {frame.total_crc_fail}",
            "#14532d" if frame.total_crc_ok > 0 else "#1e293b",
        )
        if frame.adc_clipping:
            _set_badge(self.badge_clip, f"ADC CLIP! ({frame.rx_peak_abs:.2f})", "#991b1b")
        else:
            _set_badge(self.badge_clip, f"ADC: OK ({frame.rx_peak_abs:.2f})", "#14532d")

        self.lbl_rf_metrics.setText(
            f"Car(+20k): {frame.carrier_dbfs:.1f} dBFS (|c|={frame.carrier_mag_linear:.4f}, Δ{delta_phase:+.1f}°) | "
            f"-180/+220k: {frame.lsb_peak_dbfs:.1f}/{frame.usb_peak_dbfs:.1f} | NF: {frame.noise_floor_db:.1f} dBFS"
        )

        for pkt in frame.packets:
            self._prepend_packet_row(pkt)

    def _prepend_packet_row(self, pkt: DecodedPacket) -> None:
        self.pkt_table.insertRow(0)
        ts_str = datetime.fromtimestamp(pkt.timestamp).strftime("%H:%M:%S.%f")[:-3]
        cols = [
            ts_str,
            str(pkt.seq),
            str(pkt.btn2_count),
            f"0x{pkt.payload_u16:04X}",
            pkt.payload_ascii,
            f"0x{pkt.crc_rx:02X} OK" if pkt.crc_ok else f"0x{pkt.crc_rx:02X} ERR",
            f"{pkt.snr_db:.1f}",
            pkt.raw_hex,
        ]
        for col_idx, val in enumerate(cols):
            item = QTableWidgetItem(val)
            item.setTextAlignment(Qt.AlignCenter if col_idx < 7 else Qt.AlignLeft | Qt.AlignVCenter)
            if col_idx == 5:
                item.setForeground(QColor("#4ade80" if pkt.crc_ok else "#f87171"))
            self.pkt_table.setItem(0, col_idx, item)
        while self.pkt_table.rowCount() > 100:
            self.pkt_table.removeRow(100)

    def _clear_packets(self) -> None:
        self.pkt_table.setRowCount(0)
        if self.usrp_worker is not None:
            self.usrp_worker.codec.crc_ok_count = 0
            self.usrp_worker.codec.crc_fail_count = 0

    # ---- MCU Flash & UART Controls ----
    def _run_flash(self, label: str, ps_args: List[str]) -> None:
        if self.flash_worker is not None and self.flash_worker.isRunning():
            self.append_log("Flash/Build already running...")
            return

        if self.serial_conn is not None and self.serial_conn.is_open:
            self._reconnect_com_after_flash = self.serial_conn.port
            self._close_uart()
        else:
            self._reconnect_com_after_flash = None

        self.btn_flash_pkt.setEnabled(False)
        self.btn_flash_cw.setEnabled(False)
        self.btn_erase.setEnabled(False)

        self.flash_worker = FlashWorker(label, ps_args, parent=self)
        self.flash_worker.log_line.connect(self.append_log)
        self.flash_worker.finished_status.connect(self._on_flash_done)
        self.flash_worker.start()

    def _on_flash_done(self, rc: int, label: str) -> None:
        self.btn_flash_pkt.setEnabled(True)
        self.btn_flash_cw.setEnabled(True)
        self.btn_erase.setEnabled(True)
        self.append_log(f"[{label}] exited with code {rc}")
        self.refresh_probes_and_ports()
        if rc == 0 and self._reconnect_com_after_flash:
            idx = self.cmb_port.findData(self._reconnect_com_after_flash)
            if idx >= 0:
                self.cmb_port.setCurrentIndex(idx)
                self._open_uart(self._reconnect_com_after_flash)

    def _toggle_uart(self) -> None:
        if self.serial_conn is not None and self.serial_conn.is_open:
            self._close_uart()
        else:
            port = self.cmb_port.currentData()
            if not port:
                self.append_log("Select an active COM port first (Scan USB/COM).")
                return
            self._open_uart(str(port))

    def _open_uart(self, port: str) -> None:
        try:
            self.serial_conn = Win32Serial(port, 115200)
            self._uart_rx_buf.clear()
            self.btn_uart_toggle.setText("Disconnect UART")
            _set_badge(self.badge_uart, f"UART: {port} @ 115200", "#14532d")
            self.append_log(f"Connected to {port} (115200 8N1)")
            self.serial_conn.write_bytes(b"STATUS?\n")
        except Exception as exc:
            self.append_log(f"Failed to open {port}: {exc}")
            self._close_uart()

    def _close_uart(self) -> None:
        if self.serial_conn is not None:
            self.serial_conn.close()
            self.serial_conn = None
        self.btn_uart_toggle.setText("Connect UART")
        _set_badge(self.badge_uart, "UART: Disconnected", "#475569")

    def _send_uart_cmd(self, cmd: str) -> bool:
        if self.serial_conn is not None and self.serial_conn.is_open:
            try:
                self.serial_conn.write_bytes((cmd + "\n").encode("ascii"))
                self.append_log(f"TX UART -> {cmd}")
                return True
            except Exception as exc:
                self.append_log(f"UART write error: {exc}")
                self._close_uart()
        return False

    def _send_mcu_mode(self, mode_name: str) -> None:
        sent = self._send_uart_cmd(f"MODE:{mode_name}")
        if self.chk_loopback.isChecked() and self.usrp_worker is not None:
            self.usrp_worker.loopback_state.mode = mode_name
            self.append_log(f"Loopback Mode -> {mode_name}")
        elif not sent:
            self.append_log("UART disconnected — use LaunchPad BTN-1 on hardware to switch modes standalone.")

    def _send_mcu_payload(self) -> None:
        txt = (self.edit_payload.text() or "OK").ljust(2)[:2]
        sent = self._send_uart_cmd(f"ASCII:{txt}")
        if self.chk_loopback.isChecked() and self.usrp_worker is not None:
            val = (ord(txt[0]) << 8) | ord(txt[1])
            self.usrp_worker.loopback_state.payload_u16 = val
            self.append_log(f"Loopback Payload -> '{txt}' (0x{val:04X})")
        elif not sent:
            self.append_log("UART disconnected — use LaunchPad BTN-2 on hardware to step payload standalone.")

    def _on_sim_btn2(self) -> None:
        if self.usrp_worker is not None and self.chk_loopback.isChecked():
            st = self.usrp_worker.loopback_state
            st.btn2_count = (st.btn2_count + 1) & 0xFF
            digit = st.btn2_count % 10
            st.payload_u16 = (ord("C") << 8) | ord(str(digit))
            if st.mode != "PKT":
                st.seq = 0
                st.bit_idx = 0
                st.mode = "PKT"
            self.append_log(
                f"Loopback BTN-2 pressed -> mode=PKT, count={st.btn2_count}, pay=C{digit} (0x{st.payload_u16:04X})"
            )

    def _poll_uart(self) -> None:
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            chunk = self.serial_conn.read_available(2048)
        except Exception as exc:
            self.append_log(f"UART read error: {exc}")
            self._close_uart()
            return

        if not chunk:
            return

        self._uart_rx_buf.extend(chunk)
        while b"\n" in self._uart_rx_buf:
            idx = self._uart_rx_buf.index(b"\n")
            raw_line = bytes(self._uart_rx_buf[:idx]).rstrip(b"\r").decode("ascii", errors="replace")
            del self._uart_rx_buf[: idx + 1]
            if not raw_line:
                continue
            if raw_line.startswith("POC_MCU|"):
                self.lbl_mcu_state.setText(f"MCU Telemetry: {raw_line[8:]}")
            else:
                self.append_log(f"RX UART <- {raw_line}")

    def closeEvent(self, event) -> None:
        self.load_toggle_timer.stop()
        self.uart_timer.stop()
        self._close_uart()
        self.stop_usrp_worker()
        if self.flash_worker is not None and self.flash_worker.isRunning():
            self.flash_worker.wait(1000)
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# End-to-End Automated Self-Test (--self-test)
# ---------------------------------------------------------------------------
def run_self_test() -> int:
    print("=== [Self-Test 1/4] +20 kHz Offset CW PocPacketCodec Multi-SNR Validation ===")
    for snr_db, step_tag in ((24.0, "Normal 24 dB"), (13.0, "Low-SNR 13 dB Stress")):
        codec = PocPacketCodec()
        sim = LoopbackSignalState(rng_seed=77 if snr_db > 20 else 99)
        sim.snr_db = snr_db
        sim.cfo_hz = 15.0

        decoded_all = []
        # Step through 1.6 s in 50 ms chunks, changing payload partway to match BTN-2 steps
        for chunk_idx in range(32):
            if chunk_idx == 10:
                sim.btn2_count = 1
                sim.payload_u16 = (ord("C") << 8) | ord("1")
            elif chunk_idx == 20:
                sim.btn2_count = 2
                sim.payload_u16 = (ord("Z") << 8) | ord("9")

            iq = sim.generate(50000)
            _, pkts = codec.feed_iq(iq, snr_db_est=snr_db)
            decoded_all.extend(pkts)

        print(
            f"  [{step_tag}] Decoded={len(decoded_all)} | "
            f"CRC_OK={codec.crc_ok_count} | CRC_Fail={codec.crc_fail_count}"
        )
        assert len(decoded_all) >= 15, f"Expected >= 15 packets, got {len(decoded_all)}"
        assert codec.crc_fail_count == 0, f"Unexpected CRC failures: {codec.crc_fail_count}"
        assert all(p.crc_ok for p in decoded_all), "Found non-OK packet"

        payloads_seen = {p.payload_ascii for p in decoded_all}
        assert {"OK", "C1", "Z9"}.issubset(payloads_seen), f"Missing expected payloads: {payloads_seen}"

        # Check rolling sequence continuity for consecutive decoded packets
        for a, b in zip(decoded_all, decoded_all[1:]):
            diff = (b.seq - a.seq) & 0xFF
            assert diff == 1, f"Sequence gap detected: {a.seq} -> {b.seq}"

    print("=== [Self-Test 2/4] OPEN vs LOAD50 Phase Step & CW200K Excess SNR Validation ===")
    codec_diag = PocPacketCodec()
    sim_open = LoopbackSignalState(rng_seed=101)
    sim_open.mode = "OPEN"
    sim_open.snr_db = 28.0
    iq_open = sim_open.generate(50000)
    _, _, _, _, _, _, snr_open = compute_spectrum_and_snr(iq_open)
    mag_open, ph_open = codec_diag.measure_carrier_phasor(iq_open)

    sim_load = LoopbackSignalState(rng_seed=101)
    sim_load.mode = "LOAD50"
    sim_load.snr_db = 28.0
    iq_load = sim_load.generate(50000)
    _, _, _, _, _, _, snr_load = compute_spectrum_and_snr(iq_load)
    mag_load, ph_load = codec_diag.measure_carrier_phasor(iq_load)

    delta_ph = ((ph_load - ph_open + 180.0) % 360.0) - 180.0
    sim_cw = LoopbackSignalState(rng_seed=101)
    sim_cw.mode = "CW200K"
    sim_cw.snr_db = 24.0
    iq_cw = sim_cw.generate(50000)
    _, _, _, _, _, _, snr_cw = compute_spectrum_and_snr(iq_cw)

    print(
        f"  OPEN SNR={snr_open:+.2f} dB (|c|={mag_open:.4f}, ph={ph_open:+.1f} deg) | "
        f"LOAD50 SNR={snr_load:+.2f} dB (|c|={mag_load:.4f}, Δph={delta_ph:+.1f} deg) | "
        f"CW200K SNR={snr_cw:+.2f} dB"
    )
    assert abs(snr_open) < 3.0, f"Expected ~0 dB excess SNR in OPEN, got {snr_open}"
    assert abs(snr_load) < 3.0, f"Expected ~0 dB excess SNR in LOAD50, got {snr_load}"
    assert abs(delta_ph) > 2.0, f"Expected > 2 deg phase shift in LOAD50, got {delta_ph}"
    assert snr_cw > 15.0, f"Expected > 15 dB excess SNR in CW200K, got {snr_cw}"

    print("=== [Self-Test 3/4] Win32 Struct & Hardware Discovery Checks ===")
    assert ctypes.sizeof(_DCB) == 28, f"Unexpected DCB size: {ctypes.sizeof(_DCB)}"
    assert ctypes.sizeof(_COMMTIMEOUTS) == 20, f"Unexpected COMMTIMEOUTS size: {ctypes.sizeof(_COMMTIMEOUTS)}"
    probe_info = check_xds110_usb()
    ports = list_serial_ports()
    print(f"  XDS110: {probe_info['summary']} | COM ports: {ports} | HAS_UHD={HAS_UHD}")

    print("=== [Self-Test 4/4] PyQt5 + pyqtgraph Integrated Loopback GUI Test ===", flush=True)
    app = QApplication.instance() or QApplication([])
    win = BackscatterPocWindow(auto_start_hw=False, initial_loopback=True)
    win.show()

    # Pump Qt event loop for ~650 ms so loopback UsrpWorker delivers telemetry & populates UI
    deadline = time.monotonic() + 0.65
    while time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.015)

    row_count = win.pkt_table.rowCount()
    snr_pts = len(win.snr_values)
    x_spec = win.curve_spec.getData()[0]
    x_env = win.curve_env.getData()[0]
    spec_pts = 0 if x_spec is None else len(x_spec)
    env_pts = 0 if x_env is None else len(x_env)
    print(
        f"  GUI Rendered: spec_pts={spec_pts}, env_pts={env_pts}, "
        f"snr_pts={snr_pts}, pkt_table_rows={row_count}",
        flush=True,
    )

    assert spec_pts == 16384, f"Expected 16384 spectrum bins, got {spec_pts}"
    assert env_pts == 2000, f"Expected 2000 envelope points, got {env_pts}"
    assert snr_pts >= 4, f"Expected >= 4 telemetry frames rendered, got {snr_pts}"
    assert row_count >= 3, f"Expected >= 3 decoded rows in GUI table, got {row_count}"

    win.stop_usrp_worker()
    win.close()
    app.processEvents()
    print("[PASS] All from-scratch PoC self-tests passed cleanly.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="2.4 GHz Backscatter PoC Unified Dashboard")
    parser.add_argument("--self-test", action="store_true", help="Run automated DSP + GUI self-test and exit")
    parser.add_argument("--loopback", action="store_true", help="Start in Offline Loopback Test Source mode")
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()

    app = QApplication(sys.argv)
    win = BackscatterPocWindow(auto_start_hw=not args.loopback, initial_loopback=args.loopback)
    win.show()
    if not HAS_UHD and not args.loopback:
        win.append_log(f"UHD Python import issue: {UHD_IMPORT_ERROR}")
    return int(app.exec_())


if __name__ == "__main__":
    sys.exit(main())
