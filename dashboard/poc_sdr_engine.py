"""
poc_sdr_engine.py
From-scratch DSP, 1.0 kbps PoC packet codec, native UHD USRP-2900 worker,
and Win32 XDS110 UART/USB helpers for the 2.4 GHz backscatter PoC testbed.
"""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
from ctypes import wintypes
import math
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple
import winreg

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

try:
    import uhd  # type: ignore

    HAS_UHD = True
    UHD_IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover - runtime environment dependent
    uhd = None  # type: ignore
    HAS_UHD = False
    UHD_IMPORT_ERROR = str(exc)


# ---------------------------------------------------------------------------
# Protocol & RF constants (locked 1:1 with poc_testbed/firmware/backscatter_tag.c)
# ---------------------------------------------------------------------------
CENTER_FREQ_HZ = 2_440_000_000.0
SAMPLE_RATE_HZ = 1_000_000.0
TX_CW_OFFSET_HZ = 20_000.0
SUBCARRIER_HZ = 200_000.0
USB_OFFSET_HZ = TX_CW_OFFSET_HZ + SUBCARRIER_HZ   # +220.0 kHz
LSB_OFFSET_HZ = TX_CW_OFFSET_HZ - SUBCARRIER_HZ   # -180.0 kHz
DECIMATION = 100
ENV_RATE_HZ = SAMPLE_RATE_HZ / DECIMATION  # 10.0 kS/s
BIT_RATE_BPS = 1000.0
SAMPLES_PER_BIT = int(round(ENV_RATE_HZ / BIT_RATE_BPS))  # 10 samples/bit

PREAMBLE_BYTE = 0xAA
SYNC_HI = 0xD3
SYNC_LO = 0x91
SYNC_WORD = 0xD391
MAGIC_BYTE = 0xA5
ACTIVE_FRAME_BYTES = 9    # [AA, D3, 91, A5, seq, btn2, pay_hi, pay_lo, crc8]
GUARD_BYTES = 2           # [00, 00]
TOTAL_FRAME_BYTES = ACTIVE_FRAME_BYTES + GUARD_BYTES  # 11 bytes = 88 bits (88.0 ms)
ACTIVE_FRAME_BITS = ACTIVE_FRAME_BYTES * 8            # 72 bits
TOTAL_FRAME_BITS = TOTAL_FRAME_BYTES * 8              # 88 bits

HDR_BYTES = bytes([PREAMBLE_BYTE, SYNC_HI, SYNC_LO])
HDR_BITS = np.unpackbits(np.frombuffer(HDR_BYTES, dtype=np.uint8))
HDR_ONE_OFFSETS = (np.flatnonzero(HDR_BITS == 1) * SAMPLES_PER_BIT).astype(np.int32)
HDR_ZERO_OFFSETS = (np.flatnonzero(HDR_BITS == 0) * SAMPLES_PER_BIT).astype(np.int32)
HDR_BIT_MASK = HDR_BITS == 1

XDSDFU_EXE = r"C:\ti\ccs2101\ccs\ccs_base\common\uscif\xds110\xdsdfu.exe"


def crc8_ccitt(data: bytes) -> int:
    """Compute CRC-8-CCITT (poly=0x07, init=0x00, MSB-first)."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def payload_to_ascii(payload_u16: int) -> str:
    """Render a 16-bit big-endian payload as printable ASCII or '.' placeholders."""
    hi = (payload_u16 >> 8) & 0xFF
    lo = payload_u16 & 0xFF
    ch1 = chr(hi) if 32 <= hi <= 126 else "."
    ch2 = chr(lo) if 32 <= lo <= 126 else "."
    return f"{ch1}{ch2}"


@dataclass
class DecodedPacket:
    timestamp: float
    seq: int
    btn2_count: int
    payload_u16: int
    payload_ascii: str
    crc_rx: int
    crc_ok: bool
    snr_db: float
    raw_hex: str


@dataclass
class SdrTelemetryFrame:
    timestamp: float
    source_name: str
    is_loopback: bool
    freqs_khz: np.ndarray
    spectrum_db: np.ndarray
    env_ms: np.ndarray
    env_mag: np.ndarray
    carrier_dbfs: float
    carrier_mag_linear: float
    carrier_phase_deg: float
    usb_peak_dbfs: float
    lsb_peak_dbfs: float
    noise_floor_db: float
    sideband_snr_db: float
    rx_rms_dbfs: float
    rx_peak_abs: float
    adc_clipping: bool
    packets: List[DecodedPacket]
    total_crc_ok: int
    total_crc_fail: int
    overflows: int
    underflows: int


class PocPacketCodec:
    """
    Matched-filter DSB (-180 kHz / +220 kHz around +20 kHz TX CW) demodulator
    and 88-bit 1.0 kbps frame codec.

    Channelization math at Fs = 1.0 MS/s and TX_CW_OFFSET_HZ = +20 kHz:
    100 samples span exact integer cycles for all three tones (2 cycles at +20 kHz,
    22 cycles at +220 kHz USB, -18 cycles at -180 kHz LSB), placing exact sinc nulls
    on 0 Hz LO DC leakage and the opposite tones.
    """

    def __init__(self) -> None:
        n100 = np.arange(DECIMATION, dtype=np.float64)
        self._mix_car = np.exp(-1j * 2.0 * np.pi * TX_CW_OFFSET_HZ * n100 / SAMPLE_RATE_HZ).astype(np.complex64)
        self._mix_usb = np.exp(-1j * 2.0 * np.pi * USB_OFFSET_HZ * n100 / SAMPLE_RATE_HZ).astype(np.complex64)
        self._mix_lsb = np.exp(-1j * 2.0 * np.pi * LSB_OFFSET_HZ * n100 / SAMPLE_RATE_HZ).astype(np.complex64)
        self._mf_kernel = np.full(SAMPLES_PER_BIT, 1.0 / SAMPLES_PER_BIT, dtype=np.float32)

        self._tail_usb = np.zeros(SAMPLES_PER_BIT - 1, dtype=np.complex64)
        self._tail_lsb = np.zeros(SAMPLES_PER_BIT - 1, dtype=np.complex64)
        self._env_buf = np.zeros(0, dtype=np.float32)
        self.crc_ok_count = 0
        self.crc_fail_count = 0

    def measure_carrier_phasor(self, iq: np.ndarray) -> Tuple[float, float]:
        """Measure complex magnitude and phase (deg) at +20.000 kHz offset CW."""
        usable = (len(iq) // DECIMATION) * DECIMATION
        if usable == 0:
            return 0.0, 0.0
        blocks = iq[:usable].reshape(-1, DECIMATION)
        c20 = complex(np.mean(blocks * self._mix_car))
        return abs(c20), float(math.degrees(math.atan2(c20.imag, c20.real)))

    def reset(self) -> None:
        self._tail_usb.fill(0)
        self._tail_lsb.fill(0)
        self._env_buf = np.zeros(0, dtype=np.float32)
        self.crc_ok_count = 0
        self.crc_fail_count = 0

    @staticmethod
    def encode_frame_bytes(seq: int, btn2_count: int, payload_u16: int) -> bytes:
        body = bytes(
            [
                MAGIC_BYTE,
                seq & 0xFF,
                btn2_count & 0xFF,
                (payload_u16 >> 8) & 0xFF,
                payload_u16 & 0xFF,
            ]
        )
        crc = crc8_ccitt(body)
        return HDR_BYTES + body + bytes([crc, 0x00, 0x00])

    @staticmethod
    def encode_frame_bits(seq: int, btn2_count: int, payload_u16: int) -> np.ndarray:
        frame = PocPacketCodec.encode_frame_bytes(seq, btn2_count, payload_u16)
        return np.unpackbits(np.frombuffer(frame, dtype=np.uint8))

    def iq_to_envelope(self, iq: np.ndarray) -> np.ndarray:
        """Downconvert +/-200 kHz, decimate 100:1 to 10.0 kS/s, and matched-filter."""
        usable = (len(iq) // DECIMATION) * DECIMATION
        if usable == 0:
            return np.zeros(0, dtype=np.float32)

        blocks = iq[:usable].reshape(-1, DECIMATION)
        z_usb = np.mean(blocks * self._mix_usb, axis=1).astype(np.complex64)
        z_lsb = np.mean(blocks * self._mix_lsb, axis=1).astype(np.complex64)

        padded_usb = np.concatenate((self._tail_usb, z_usb))
        padded_lsb = np.concatenate((self._tail_lsb, z_lsb))
        self._tail_usb = z_usb[-(SAMPLES_PER_BIT - 1) :].copy()
        self._tail_lsb = z_lsb[-(SAMPLES_PER_BIT - 1) :].copy()

        mf_usb = np.convolve(padded_usb, self._mf_kernel, mode="valid")
        mf_lsb = np.convolve(padded_lsb, self._mf_kernel, mode="valid")
        env = np.sqrt(
            np.real(mf_usb) ** 2 + np.imag(mf_usb) ** 2 + np.real(mf_lsb) ** 2 + np.imag(mf_lsb) ** 2
        )
        return env.astype(np.float32)

    def feed_iq(self, iq: np.ndarray, snr_db_est: float, timestamp: Optional[float] = None) -> Tuple[np.ndarray, List[DecodedPacket]]:
        """Process an IQ block (len multiple of 100) and decode any complete frames."""
        ts = time.time() if timestamp is None else timestamp
        env = self.iq_to_envelope(iq)
        if len(env) == 0:
            return env, []

        self._env_buf = np.concatenate((self._env_buf, env))
        packets = self._decode_buffer(snr_db_est=snr_db_est, timestamp=ts)
        return env, packets

    def _decode_buffer(self, snr_db_est: float, timestamp: float) -> List[DecodedPacket]:
        packets: List[DecodedPacket] = []
        n = len(self._env_buf)
        needed = ACTIVE_FRAME_BITS * SAMPLES_PER_BIT  # 720 samples
        limit = n - needed + 1
        if limit <= 0:
            return packets

        # Vectorized normalized bipolar correlation across the 24-bit header (0xAA D3 91)
        idx = np.arange(limit, dtype=np.int32)[:, None]
        ones_avg = np.mean(self._env_buf[idx + HDR_ONE_OFFSETS[None, :]], axis=1)
        zeros_avg = np.mean(self._env_buf[idx + HDR_ZERO_OFFSETS[None, :]], axis=1)
        norm_score = (ones_avg - zeros_avg) / np.maximum(ones_avg + zeros_avg, 1e-12)

        cursor = 0
        consumed = 0
        while cursor < limit:
            if norm_score[cursor] < 0.16:
                cursor += 1
                continue

            win_end = min(cursor + SAMPLES_PER_BIT, limit)
            local_p = cursor + int(np.argmax(norm_score[cursor:win_end]))

            decoded_pkt, near_sync_hit = self._try_slice_window(local_p, limit, snr_db_est, timestamp)
            if decoded_pkt is not None:
                packets.append(decoded_pkt)
                self.crc_ok_count += 1
                advance = local_p + 76 * SAMPLES_PER_BIT
                consumed = max(consumed, min(advance, n))
                cursor = advance
            elif near_sync_hit:
                self.crc_fail_count += 1
                advance = local_p + 72 * SAMPLES_PER_BIT
                consumed = max(consumed, min(advance, n))
                cursor = advance
            else:
                cursor = win_end

        if consumed > 0:
            self._env_buf = self._env_buf[consumed:].copy()
        elif len(self._env_buf) > 1600:
            self._env_buf = self._env_buf[-needed:].copy()

        return packets

    def _try_slice_window(
        self,
        center_p: int,
        limit: int,
        snr_db_est: float,
        timestamp: float,
    ) -> Tuple[Optional[DecodedPacket], bool]:
        near_sync_hit = False
        offsets = [0, -1, 1, -2, 2]
        for off in offsets:
            p = center_p + off
            if p < 0 or p >= limit:
                continue

            s72 = self._env_buf[p : p + ACTIVE_FRAME_BITS * SAMPLES_PER_BIT : SAMPLES_PER_BIT]
            hdr24 = s72[:24]
            one_m = float(np.mean(hdr24[HDR_BIT_MASK]))
            zero_m = float(np.mean(hdr24[~HDR_BIT_MASK]))
            if one_m <= zero_m * 1.25:
                continue

            thr = 0.5 * (one_m + zero_m)
            bits72 = (s72 > thr).astype(np.uint8)
            frame_bytes = np.packbits(bits72).tobytes()

            sync_val = (frame_bytes[1] << 8) | frame_bytes[2]
            magic_val = frame_bytes[3]
            if sync_val != SYNC_WORD or magic_val != MAGIC_BYTE:
                continue

            near_sync_hit = True
            crc_rx = frame_bytes[8]
            crc_calc = crc8_ccitt(frame_bytes[3:8])
            if crc_rx == crc_calc:
                pay_u16 = (frame_bytes[6] << 8) | frame_bytes[7]
                pkt = DecodedPacket(
                    timestamp=timestamp,
                    seq=frame_bytes[4],
                    btn2_count=frame_bytes[5],
                    payload_u16=pay_u16,
                    payload_ascii=payload_to_ascii(pay_u16),
                    crc_rx=crc_rx,
                    crc_ok=True,
                    snr_db=snr_db_est,
                    raw_hex=frame_bytes.hex().upper(),
                )
                return pkt, False

        return None, near_sync_hit


class LoopbackSignalState:
    """Deterministic offline RF loopback synthesizer for self-test & UI verification."""

    def __init__(self, rng_seed: int = 2026) -> None:
        self.rng = np.random.default_rng(rng_seed)
        self.mode = "PKT"
        self.seq = 0
        self.btn2_count = 0
        self.payload_u16 = 0x4F4B  # "OK"
        self.bit_idx = 0
        self.sub_sample_idx = 0
        self.beacon_ms = 0
        self.sample_counter = 0
        self.snr_db = 24.0
        self.cfo_hz = 12.0
        self.current_bits = PocPacketCodec.encode_frame_bits(self.seq, self.btn2_count, self.payload_u16)

    def generate(self, num_samples: int, tx_enabled: bool = True) -> np.ndarray:
        samples_per_ms = int(SAMPLE_RATE_HZ / BIT_RATE_BPS)  # 1000 samples per ms
        n_ms = num_samples // samples_per_ms
        mod_01 = np.zeros(num_samples, dtype=np.float32)
        mode_up = self.mode.upper()

        for ms_idx in range(n_ms):
            start = ms_idx * samples_per_ms
            end = start + samples_per_ms
            if mode_up in ("OPEN", "OFF", "LOAD50"):
                bit = 0
            elif mode_up in ("CW", "CW200K"):
                bit = 1
            elif mode_up == "BEACON":
                bit = 1 if self.beacon_ms < 50 else 0
                self.beacon_ms = (self.beacon_ms + 1) % 100
            else:
                if self.bit_idx == 0:
                    self.current_bits = PocPacketCodec.encode_frame_bits(
                        self.seq, self.btn2_count, self.payload_u16
                    )
                    self.seq = (self.seq + 1) & 0xFF
                bit = int(self.current_bits[self.bit_idx])
                self.bit_idx = (self.bit_idx + 1) % TOTAL_FRAME_BITS
            mod_01[start:end] = float(bit)

        rem = num_samples - n_ms * samples_per_ms
        if rem > 0:
            mod_01[n_ms * samples_per_ms :] = mod_01[max(0, n_ms * samples_per_ms - 1)]

        n_vec = np.arange(self.sample_counter, self.sample_counter + num_samples, dtype=np.float64)
        self.sample_counter += num_samples

        # Offset CW illumination at +20.000 kHz
        cw_tone = np.exp(1j * 2.0 * np.pi * TX_CW_OFFSET_HZ * n_vec / SAMPLE_RATE_HZ).astype(np.complex64)

        tag_amp = 0.022
        sideband_rms = tag_amp * (1.0 / np.pi) * math.sqrt(0.6)
        noise_std = sideband_rms / (10.0 ** (self.snr_db / 20.0)) * math.sqrt(25.0)
        noise = (
            self.rng.normal(0.0, noise_std / math.sqrt(2.0), num_samples)
            + 1j * self.rng.normal(0.0, noise_std / math.sqrt(2.0), num_samples)
        ).astype(np.complex64)

        if not tx_enabled:
            return noise

        # Direct carrier leakage at +20 kHz; LOAD50 shifts static S11 (+6 deg, -0.4 dB)
        leak_complex = (0.370 + 0.050j) * cw_tone
        if mode_up == "LOAD50":
            leak_complex = leak_complex * np.complex64(0.955 * np.exp(1j * math.radians(6.0)))

        # Hardware 200 kHz square wave gated cleanly on DIO21
        sq_phase = (SUBCARRIER_HZ * n_vec / SAMPLE_RATE_HZ) % 1.0
        sq_01 = (sq_phase < 0.5).astype(np.float32)
        tag_rf_env = mod_01 * sq_01

        cfo_rot = np.exp(1j * 2.0 * np.pi * self.cfo_hz * n_vec / SAMPLE_RATE_HZ).astype(np.complex64)
        sig = leak_complex + (tag_amp * tag_rf_env * cw_tone * cfo_rot)
        return (sig + noise).astype(np.complex64)


def compute_spectrum_and_snr(iq: np.ndarray, fft_size: int = 16384) -> Tuple[np.ndarray, np.ndarray, float, float, float, float, float]:
    """Compute centered dBFS PSD (-500..+500 kHz) and calibrated -180/+220 kHz sideband SNR."""
    if len(iq) < fft_size:
        seg = np.pad(iq, (0, fft_size - len(iq)))
    else:
        seg = iq[-fft_size:]

    win = np.hanning(fft_size).astype(np.float32)
    win_norm = float(np.sum(win))
    fft_vals = np.fft.fftshift(np.fft.fft(seg * win)) / max(win_norm, 1e-12)
    # Scale by 2 so a unit-amplitude complex tone reads 0 dBFS
    lin_amp = np.maximum(np.abs(fft_vals) * 2.0, 1e-10)
    lin_pow = lin_amp ** 2
    psd_db = (10.0 * np.log10(np.maximum(lin_pow, 1e-20))).astype(np.float32)
    freqs_khz = (np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / SAMPLE_RATE_HZ)) / 1e3).astype(np.float32)

    carrier_mask = np.abs(freqs_khz - (TX_CW_OFFSET_HZ / 1e3)) <= 2.5
    usb_mask = np.abs(freqs_khz - (USB_OFFSET_HZ / 1e3)) <= 1.5
    lsb_mask = np.abs(freqs_khz - (LSB_OFFSET_HZ / 1e3)) <= 1.5
    noise_mask = (
        ((freqs_khz >= 120.0) & (freqs_khz <= 170.0))
        | ((freqs_khz >= 270.0) & (freqs_khz <= 320.0))
        | ((freqs_khz >= -320.0) & (freqs_khz <= -270.0))
        | ((freqs_khz >= -170.0) & (freqs_khz <= -120.0))
    )

    carrier_dbfs = float(np.max(psd_db[carrier_mask]))
    usb_peak = float(np.max(psd_db[usb_mask]))
    lsb_peak = float(np.max(psd_db[lsb_mask]))

    p_noise = float(np.mean(lin_pow[noise_mask]))
    p_usb = float(np.mean(lin_pow[usb_mask]))
    p_lsb = float(np.mean(lin_pow[lsb_mask]))
    noise_floor = 10.0 * math.log10(max(p_noise, 1e-20))
    snr_usb_db = 10.0 * math.log10(max(p_usb / max(p_noise, 1e-20), 1e-12))
    snr_lsb_db = 10.0 * math.log10(max(p_lsb / max(p_noise, 1e-20), 1e-12))
    snr_db = max(snr_usb_db, snr_lsb_db)
    return freqs_khz, psd_db, carrier_dbfs, usb_peak, lsb_peak, noise_floor, snr_db


# ---------------------------------------------------------------------------
# Native UHD USRP-2900 QThread Worker
# ---------------------------------------------------------------------------
class UsrpWorker(QThread):
    """
    Native uhd.usrp.MultiUSRP streaming thread for simultaneous CW TX and RX2 capture.

    Strictly refuses silent fallback to simulation: hardware mode attempts to open a real
    NI USRP-2900 / B200 (`type=b200`) and reports exact UHD/USB connection errors unless
    `loopback_test=True` is explicitly selected by the user or `--self-test`.
    """

    telemetry_ready = pyqtSignal(object)
    status_changed = pyqtSignal(str, bool)  # (message, is_connected)
    error_occurred = pyqtSignal(str)
    autotune_finished = pyqtSignal(float, float, float)  # (tx_gain, rx_gain, best_snr)

    def __init__(
        self,
        center_freq_hz: float = CENTER_FREQ_HZ,
        tx_gain_db: float = 68.0,
        rx_gain_db: float = 45.0,
        tx_cw_enabled: bool = True,
        loopback_test: bool = False,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._center_freq_hz = float(center_freq_hz)
        self._tx_gain_db = float(tx_gain_db)
        self._rx_gain_db = float(rx_gain_db)
        self._tx_cw_enabled = bool(tx_cw_enabled)
        self._loopback_test = bool(loopback_test)
        self._config_dirty = False
        self._autotune_requested = False

        self.codec = PocPacketCodec()
        self.loopback_state = LoopbackSignalState()
        self._env_history = np.zeros(int(0.2 * ENV_RATE_HZ), dtype=np.float32)  # 200 ms = 2000 pts
        self._env_ms_axis = np.linspace(-200.0, 0.0, len(self._env_history), endpoint=False, dtype=np.float32)

    def set_rf_params(
        self,
        center_freq_hz: Optional[float] = None,
        tx_gain_db: Optional[float] = None,
        rx_gain_db: Optional[float] = None,
        tx_cw_enabled: Optional[bool] = None,
    ) -> None:
        with self._lock:
            if center_freq_hz is not None:
                self._center_freq_hz = float(center_freq_hz)
            if tx_gain_db is not None:
                self._tx_gain_db = float(tx_gain_db)
            if rx_gain_db is not None:
                self._rx_gain_db = float(rx_gain_db)
            if tx_cw_enabled is not None:
                self._tx_cw_enabled = bool(tx_cw_enabled)
            self._config_dirty = True

    def request_auto_tune(self) -> None:
        with self._lock:
            self._autotune_requested = True

    def stop_worker(self, wait_ms: int = 3000) -> None:
        self._stop_event.set()
        self.wait(wait_ms)

    def _push_env_history(self, new_env: np.ndarray) -> np.ndarray:
        m = len(new_env)
        if m == 0:
            return self._env_history.copy()
        if m >= len(self._env_history):
            self._env_history[:] = new_env[-len(self._env_history) :]
        else:
            self._env_history[:-m] = self._env_history[m:]
            self._env_history[-m:] = new_env
        return self._env_history.copy()

    def run(self) -> None:
        self.codec.reset()
        with self._lock:
            loopback = self._loopback_test

        if loopback:
            self._run_loopback_loop()
        else:
            self._run_hardware_loop()

    def _run_loopback_loop(self) -> None:
        self.status_changed.emit("Offline Loopback Test Source active (no RF hardware)", True)
        block_samples = 50000  # 50 ms at 1.0 MS/s
        next_tick = time.perf_counter()

        while not self._stop_event.is_set():
            with self._lock:
                tx_on = self._tx_cw_enabled
                if self._autotune_requested:
                    self._autotune_requested = False
                    self._tx_gain_db = 52.0
                    self._rx_gain_db = 48.0
                    self.autotune_finished.emit(52.0, 48.0, 24.0)

            iq = self.loopback_state.generate(block_samples, tx_enabled=tx_on)
            self._process_and_emit(iq, source_name="Offline Loopback Test", is_loopback=True, ovf=0, unf=0)

            next_tick += 0.050
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.001:
                self._stop_event.wait(sleep_s)
            else:
                next_tick = time.perf_counter()

        self.status_changed.emit("Offline Loopback stopped", False)

    def _run_hardware_loop(self) -> None:
        if not HAS_UHD or uhd is None:
            self.error_occurred.emit(f"UHD Python module unavailable: {UHD_IMPORT_ERROR}")
            self.status_changed.emit("UHD Python unavailable", False)
            return

        usrp = None
        rx_streamer = None
        tx_streamer = None
        tx_md = None
        ovf_count = 0
        unf_count = 0

        try:
            found = uhd.find("type=b200")
            if not found:
                msg = (
                    "No NI USRP-2900 / B200 found by UHD (uhd.find returned []). "
                    "If Windows Device Manager shows the USRP present after an earlier crash, "
                    "physically unplug and replug the USB 3.0 cable."
                )
                self.error_occurred.emit(msg)
                self.status_changed.emit("USRP-2900 disconnected (uhd.find empty)", False)
                return

            self.status_changed.emit("Opening NI USRP-2900 (type=b200)...", False)
            usrp = uhd.usrp.MultiUSRP("type=b200")
            usrp.set_rx_rate(SAMPLE_RATE_HZ, 0)
            usrp.set_tx_rate(SAMPLE_RATE_HZ, 0)
            usrp.set_rx_antenna("RX2", 0)
            usrp.set_tx_antenna("TX/RX", 0)
            usrp.set_rx_agc(False, 0)

            self._apply_usrp_rf(usrp)

            st_args = uhd.usrp.StreamArgs("fc32", "sc16")
            st_args.channels = [0]
            rx_streamer = usrp.get_rx_stream(st_args)
            tx_streamer = usrp.get_tx_stream(st_args)

            chunk_samples = 5000  # 5 ms per USB transfer (multiple of 100)
            chunks_per_frame = 10  # 50 ms telemetry/decode cadence
            rx_buf = np.zeros((1, chunk_samples), dtype=np.complex64)
            iq_acc = np.zeros(chunk_samples * chunks_per_frame, dtype=np.complex64)
            n_tx = np.arange(chunk_samples, dtype=np.float64)
            cw_buf = (
                0.707 * np.exp(1j * 2.0 * np.pi * TX_CW_OFFSET_HZ * n_tx / SAMPLE_RATE_HZ)
            ).astype(np.complex64)[None, :]
            zero_buf = np.zeros((1, chunk_samples), dtype=np.complex64)

            rx_md = uhd.types.RXMetadata()
            tx_md = uhd.types.TXMetadata()
            tx_md.start_of_burst = True
            tx_md.end_of_burst = False

            # Prefill TX buffer slightly before starting RX
            with self._lock:
                tx_on = self._tx_cw_enabled
            for _ in range(3):
                tx_streamer.send(cw_buf if tx_on else zero_buf, tx_md, timeout=0.1)
                tx_md.start_of_burst = False

            start_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
            start_cmd.stream_now = True
            rx_streamer.issue_stream_cmd(start_cmd)

            self.status_changed.emit("NI USRP-2900 streaming live @ 2.44 GHz", True)

            tune_candidates: List[Tuple[float, float]] = [
                (55.0, 38.0),
                (62.0, 42.0),
                (68.0, 45.0),
                (72.0, 45.0),
                (74.0, 48.0),
                (76.0, 42.0),
            ]
            tune_idx = -1
            tune_best: Tuple[float, float, float] = (68.0, 45.0, -99.0)

            acc_idx = 0
            while not self._stop_event.is_set():
                with self._lock:
                    dirty = self._config_dirty
                    self._config_dirty = False
                    tx_on = self._tx_cw_enabled
                    if self._autotune_requested and tune_idx < 0:
                        self._autotune_requested = False
                        tune_idx = 0
                        tune_best = (self._tx_gain_db, self._rx_gain_db, -99.0)
                        cand_tx, cand_rx = tune_candidates[0]
                        self._tx_gain_db = cand_tx
                        self._rx_gain_db = cand_rx
                        dirty = True

                if dirty:
                    self._apply_usrp_rf(usrp)

                tx_streamer.send(cw_buf if tx_on else zero_buf, tx_md, timeout=0.1)
                n_rx = rx_streamer.recv(rx_buf, rx_md, timeout=0.2)
                if rx_md.error_code == uhd.types.RXMetadataErrorCode.overflow:
                    ovf_count += 1
                if n_rx <= 0:
                    continue

                if n_rx == chunk_samples and acc_idx + chunk_samples <= len(iq_acc):
                    iq_acc[acc_idx : acc_idx + chunk_samples] = rx_buf[0]
                    acc_idx += chunk_samples

                if acc_idx >= len(iq_acc):
                    frame = self._process_and_emit(
                        iq_acc,
                        source_name="NI USRP-2900",
                        is_loopback=False,
                        ovf=ovf_count,
                        unf=unf_count,
                    )
                    acc_idx = 0

                    if tune_idx >= 0:
                        cand_tx, cand_rx = tune_candidates[tune_idx]
                        if (not frame.adc_clipping) and frame.sideband_snr_db > tune_best[2]:
                            tune_best = (cand_tx, cand_rx, frame.sideband_snr_db)
                        tune_idx += 1
                        if tune_idx < len(tune_candidates):
                            ntx, nrx = tune_candidates[tune_idx]
                            with self._lock:
                                self._tx_gain_db = ntx
                                self._rx_gain_db = nrx
                                self._config_dirty = True
                        else:
                            btx, brx, bsnr = tune_best
                            with self._lock:
                                self._tx_gain_db = btx
                                self._rx_gain_db = brx
                                self._config_dirty = True
                            self.autotune_finished.emit(btx, brx, bsnr)
                            tune_idx = -1

        except Exception as exc:
            self.error_occurred.emit(f"USRP-2900 stream error: {exc}")
            self.status_changed.emit(f"USRP error: {exc}", False)
        finally:
            try:
                if rx_streamer is not None:
                    stop_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont)
                    rx_streamer.issue_stream_cmd(stop_cmd)
            except Exception:
                pass
            try:
                if tx_streamer is not None and tx_md is not None:
                    tx_md.end_of_burst = True
                    tx_streamer.send(np.zeros((1, 16), dtype=np.complex64), tx_md, timeout=0.05)
            except Exception:
                pass
            usrp = None
            rx_streamer = None
            tx_streamer = None
            self.status_changed.emit("USRP-2900 disconnected", False)

    def _apply_usrp_rf(self, usrp) -> None:
        with self._lock:
            freq = self._center_freq_hz
            tx_g = self._tx_gain_db if self._tx_cw_enabled else 0.0
            rx_g = self._rx_gain_db
        usrp.set_rx_freq(uhd.types.TuneRequest(freq), 0)
        usrp.set_tx_freq(uhd.types.TuneRequest(freq), 0)
        usrp.set_tx_gain(tx_g, 0)
        usrp.set_rx_gain(rx_g, 0)

    def _process_and_emit(
        self,
        iq: np.ndarray,
        source_name: str,
        is_loopback: bool,
        ovf: int,
        unf: int,
    ) -> SdrTelemetryFrame:
        ts = time.time()
        freqs_khz, psd_db, car_db, usb_db, lsb_db, nf_db, snr_db = compute_spectrum_and_snr(iq)
        rms = float(np.sqrt(np.mean(np.abs(iq) ** 2)))
        rx_rms_db = 20.0 * math.log10(max(rms, 1e-10))
        peak_abs = float(np.max(np.abs(iq)))
        clipping = peak_abs > 0.90

        car_mag, car_phase_deg = self.codec.measure_carrier_phasor(iq)
        env, pkts = self.codec.feed_iq(iq, snr_db_est=snr_db, timestamp=ts)
        env_hist = self._push_env_history(env)

        frame = SdrTelemetryFrame(
            timestamp=ts,
            source_name=source_name,
            is_loopback=is_loopback,
            freqs_khz=freqs_khz,
            spectrum_db=psd_db,
            env_ms=self._env_ms_axis,
            env_mag=env_hist,
            carrier_dbfs=car_db,
            carrier_mag_linear=car_mag,
            carrier_phase_deg=car_phase_deg,
            usb_peak_dbfs=usb_db,
            lsb_peak_dbfs=lsb_db,
            noise_floor_db=nf_db,
            sideband_snr_db=snr_db,
            rx_rms_dbfs=rx_rms_db,
            rx_peak_abs=peak_abs,
            adc_clipping=clipping,
            packets=pkts,
            total_crc_ok=self.codec.crc_ok_count,
            total_crc_fail=self.codec.crc_fail_count,
            overflows=ovf,
            underflows=unf,
        )
        self.telemetry_ready.emit(frame)
        return frame


# ---------------------------------------------------------------------------
# Pure Win32 ctypes Serial + XDS110 Probe Detection (no pyserial needed)
# ---------------------------------------------------------------------------
class _DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD),
        ("BaudRate", wintypes.DWORD),
        ("flags", wintypes.DWORD),
        ("wReserved", wintypes.WORD),
        ("XonLim", wintypes.WORD),
        ("XoffLim", wintypes.WORD),
        ("ByteSize", wintypes.BYTE),
        ("Parity", wintypes.BYTE),
        ("StopBits", wintypes.BYTE),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wintypes.WORD),
    ]


class _COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wintypes.DWORD),
        ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
        ("ReadTotalTimeoutConstant", wintypes.DWORD),
        ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
        ("WriteTotalTimeoutConstant", wintypes.DWORD),
    ]


class Win32Serial:
    """Minimal non-blocking Win32 COM port reader/writer using ctypes."""

    _GENERIC_READ_WRITE = 0x80000000 | 0x40000000
    _OPEN_EXISTING = 3
    _INVALID_HANDLE = wintypes.HANDLE(-1).value

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self._handle: Optional[int] = None
        self._kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self._open()

    def _open(self) -> None:
        dev_path = self.port if self.port.startswith(r"\\.") else rf"\\.\{self.port}"
        handle = self._kernel32.CreateFileW(
            dev_path,
            self._GENERIC_READ_WRITE,
            0,
            None,
            self._OPEN_EXISTING,
            0,
            None,
        )
        if handle == self._INVALID_HANDLE or handle is None:
            err = ctypes.get_last_error()
            raise OSError(err, f"Failed to open {self.port} (Win32 error {err})")

        try:
            dcb = _DCB()
            dcb.DCBlength = ctypes.sizeof(_DCB)
            if not self._kernel32.GetCommState(handle, ctypes.byref(dcb)):
                raise OSError(ctypes.get_last_error(), "GetCommState failed")

            dcb.BaudRate = self.baudrate
            dcb.flags = 0x00000001  # fBinary=1, no flow control, DTR=0, RTS=0
            dcb.ByteSize = 8
            dcb.Parity = 0
            dcb.StopBits = 0
            if not self._kernel32.SetCommState(handle, ctypes.byref(dcb)):
                raise OSError(ctypes.get_last_error(), "SetCommState failed")

            timeouts = _COMMTIMEOUTS(
                ReadIntervalTimeout=0xFFFFFFFF,
                ReadTotalTimeoutMultiplier=0,
                ReadTotalTimeoutConstant=0,
                WriteTotalTimeoutMultiplier=0,
                WriteTotalTimeoutConstant=250,
            )
            if not self._kernel32.SetCommTimeouts(handle, ctypes.byref(timeouts)):
                raise OSError(ctypes.get_last_error(), "SetCommTimeouts failed")

            # Purge stale RX/TX bytes
            self._kernel32.PurgeComm(handle, 0x000F)
        except Exception:
            self._kernel32.CloseHandle(handle)
            raise

        self._handle = handle

    @property
    def is_open(self) -> bool:
        return self._handle is not None

    def read_available(self, max_bytes: int = 2048) -> bytes:
        if self._handle is None:
            return b""
        buf = ctypes.create_string_buffer(max_bytes)
        read_len = wintypes.DWORD(0)
        ok = self._kernel32.ReadFile(self._handle, buf, max_bytes, ctypes.byref(read_len), None)
        if not ok:
            raise OSError(ctypes.get_last_error(), f"ReadFile failed on {self.port}")
        return buf.raw[: read_len.value]

    def write_bytes(self, data: bytes) -> int:
        if self._handle is None:
            raise OSError("Serial port is not open")
        written = wintypes.DWORD(0)
        ok = self._kernel32.WriteFile(self._handle, data, len(data), ctypes.byref(written), None)
        if not ok:
            raise OSError(ctypes.get_last_error(), f"WriteFile failed on {self.port}")
        return int(written.value)

    def close(self) -> None:
        if self._handle is not None:
            try:
                self._kernel32.CloseHandle(self._handle)
            finally:
                self._handle = None


def list_serial_ports() -> List[Tuple[str, str]]:
    """Enumerate active COM ports from Windows registry SERIALCOMM."""
    ports: List[Tuple[str, str]] = []
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
            idx = 0
            while True:
                try:
                    name, val, _ = winreg.EnumValue(key, idx)
                    ports.append((str(val), str(name)))
                    idx += 1
                except OSError:
                    break
    except OSError:
        pass
    ports.sort(key=lambda item: (len(item[0]), item[0]))
    return ports


def check_xds110_usb() -> Dict[str, object]:
    """Query TI xdsdfu.exe and active COM ports to inspect LAUNCHXL XDS110 presence."""
    ports = list_serial_ports()
    dfu_present = False
    summary = "XDS110 check not run"
    try:
        proc = subprocess.run(
            [XDSDFU_EXE, "-e"],
            capture_output=True,
            text=True,
            timeout=4,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        out = (proc.stdout + "\n" + proc.stderr).strip()
        if "Found 0 devices" in out:
            dfu_present = False
            summary = "XDS110 Debug Probe NOT enumerated (Found 0 devices)"
        else:
            dfu_present = True
            summary = "XDS110 Debug Probe detected"
    except Exception as exc:
        summary = f"XDS110 probe check error: {exc}"

    return {"present": dfu_present, "summary": summary, "ports": ports}
