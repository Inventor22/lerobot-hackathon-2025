# lerobot/src/classes/marlin_gcode_motor_bus.py
"""A MotorBus implementation that talks **Marlin G‑code** instead of Feetech's
half‑duplex serial protocol.  It keeps the public surface that the rest of the
LeRobot stack expects (read / write / connect / disconnect / calibration helpers)
but translates the high‑level calls into plain G‑code commands understood by a
BigTreeTech SKR‑Mini (or any Marlin‑compatible) controller.

Only the subset of features actually used by the CoreXY Stylus robot is
implemented.  Anything exotic (EEPROM writes, sync‑read, sign‑magnitude
encoding, etc.) raises *NotImplementedError* so you notice immediately if the
code path is hit.
"""
from __future__ import annotations

import logging
import re
import threading
import time
from typing import Dict, Tuple, Any, Optional

import serial  # pyserial

# LeRobot base abstractions (required)
from lerobot.common.motors import (
    Motor,
    MotorCalibration,
    MotorsBus,
    NameOrID,
    Value,
)

logger = logging.getLogger(__name__)

# Default settings for a BigTreeTech SKR‑Mini (Marlin)
DEFAULT_BAUDRATE = 115_200
DEFAULT_TIMEOUT_S = 1.0

# Regex used to parse `M114` position reports (Marlin flavour)
_POS_RE = re.compile(r"([XYZE]):([-+]?\d*\.\d+|[-+]?\d+)")


class MarlinGCodeMotorBus(MotorsBus):
    """Thin adapter that converts the *MotorsBus* abstraction into Marlin G‑code.

    Notes
    -----
    *   We run one small read‑lock so that *write* / *read* / *get_action* calls
        coming from different threads don’t stomp on the serial traffic.
    *   Only the addresses used by *CoreXYStylusRobot* are implemented – mainly
        "Goal_Position" (write) and "Present_Position" (read).
        Anything else raises *NotImplementedError* so bugs surface early.
    """

    available_baudrates = [115_200, 250_000, 1_000_000]
    default_baudrate = DEFAULT_BAUDRATE
    default_timeout = DEFAULT_TIMEOUT_S

    def __init__(
        self,
        port: str,
        motors: Dict[str, Motor],
        calibration: Dict[str, MotorCalibration] | None = None,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT_S,
    ) -> None:
        super().__init__(port, motors, calibration)
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

    # ── lifecycle ──────────────────────────────────────────────────────────
    def connect(self) -> None:
        if self.ser and self.ser.is_open:
            return  # already connected

        logger.info("Opening serial %s @ %d…", self.port, self.baudrate)
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2)  # bootloader‑style reset
        self._flush_buffers()

        # Absolute positioning (G90) + millimetres (G21)
        self._send("G90")
        self._send("G21")
        logger.info("Marlin bus connected.")

    def disconnect(self) -> None:
        if self.ser and self.ser.is_open:
            logger.info("Closing serial %s…", self.port)
            self.ser.close()
        self.ser = None

    # ── internal helpers ──────────────────────────────────────────────────
    def _flush_buffers(self) -> None:
        assert self.ser, "Serial not open"
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def _await_ok(self) -> str:
        """Reads until Marlin sends an `ok`. Returns the last full line received
        **before** the ok (useful to grab data from commands like `M114`)."""
        assert self.ser, "Serial not open"
        last_line = ""
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("ok") or line == "ok":
                return last_line
            last_line = line  # keep updating until we hit the ok

    def _send(self, gcode: str) -> str:
        """Thread‑safe send; returns the line right before `ok`."""
        with self._lock:
            assert self.ser, "Serial not open"
            self.ser.write(f"{gcode}\n".encode())
            self.ser.flush()
            return self._await_ok()
        
    def _tap(self, tap_time=300):
        # solenoid on pin 41
        self._send('M42 P41 S255')
        time.sleep(tap_time / 1000)
        self._send('M42 P41 S0')

        # Dampening pulse, to avoid stylus from bouncing/double tapping
        time.sleep(0.005)        
        self._send('M42 P41 S255')
        time.sleep(0.005)
        self._send('M42 P41 S0')

    # ── public API expected by LeRobot ────────────────────────────────────

    # ---------- write -----------------------------------------------------
    def write(
        self,
        data_name: str,
        motor: NameOrID | None,
        value: Value,
        *,
        normalize: bool = True,
        num_retry: int = 0,
        **kwargs: Any,
    ) -> None:
        """Map *data_name* to a G‑code command.

        Supported:
        *   "Goal_Position"  → `G0/G1` move on the requested axis.
        *   "Torque_Enable"  → `M17` (enable) / `M18` (disable)
        """
        if data_name == "Goal_Position":
            axis = str(motor).upper()
            if axis not in {"X", "Y", "TAP"}:
                raise ValueError(f"Unsupported axis '{axis}' for Goal_Position")
            if axis == "TAP" and value > 0:
                self._tap()
            else:
                self._send(f"G0 {axis}{float(value):.3f}")
            return

        if data_name == "Torque_Enable":
            self._send("M17" if int(value) else "M18")
            return

        raise NotImplementedError(f"write(): unsupported data_name '{data_name}'")

    # ---------- read ------------------------------------------------------
    def read(
        self,
        data_name: str,
        motor: NameOrID | None,
        *,
        normalize: bool = True,
        num_retry: int = 0,
        **kwargs: Any,
    ) -> Value:
        """Reads data back from the board.

        Supported:
        *   "Present_Position" → uses `M114` and extracts the requested axis.
        """
        if data_name == "Present_Position":
            axis = str(motor).upper()
            if axis not in {"X", "Y", "Z", "E"}:
                raise ValueError(f"Unsupported axis '{axis}' for Present_Position")

            last = self._send("M114")
            match = _POS_RE.search(last)
            if not match:
                raise RuntimeError(f"Could not parse position from line: '{last}'")

            return {k: float(v) for k, v in _POS_RE.findall(last)}.get(axis, 0.0)

        raise NotImplementedError(f"read(): unsupported data_name '{data_name}'")

    # ---------- calibration helpers --------------------------------------
    @property
    def is_calibrated(self) -> bool:
        return True  # Marlin doesn’t store limits per stepper in EEPROM

    def read_calibration(self) -> Dict[str, MotorCalibration]:
        return self.calibration

    def write_calibration(self, calibration_dict: Dict[str, MotorCalibration]) -> None:
        # Not sure if Marlin can store calibration limits in EEPROM, will explore later
        self.calibration = calibration_dict

    # ---------- torque convenience wrappers ------------------------------
    def disable_torque(self, *args, **kwargs):
        self._send("M18")

    def enable_torque(self, *args, **kwargs):
        self._send("M17")

    # ---------- unimplemented rich API -----------------------------------
    def configure_motors(self, *args, **kwargs):
        logger.debug("configure_motors(): nothing to do for Marlin board")

    # Expose raw G‑code send for convenience
    # ---------------------------------------------------------------------
    def send_gcode(self, line: str) -> str:
        """Send arbitrary G‑code and return the response line before `ok`."""
        return self._send(line)
