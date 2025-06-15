# lerobot/src/classes/marlin_gcode_motor_bus.py

from __future__ import annotations
import serial
import time

class MarlinGCodeMotorBus:
    """Thin wrapper that turns LeRobot 'bus' calls into Marlin G-code."""

    def __init__(self, port: str = "COM5", baud: int = 115200, timeout: float = 1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: serial.Serial | None = None
        self.is_connected: bool = False

    # ── lifecycle ────────────────────────────────────────────────────────────
    def connect(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2)  # board reboots on USB open
        self.is_connected = True
        self._flush_buffers()
        self.write("G90")  # absolute-pos
        self.write("G21")  # mm units

    def disconnect(self) -> None:
        if self.ser:
            self.ser.close()
        self.is_connected = False

    # ── helpers ──────────────────────────────────────────────────────────────
    def _flush_buffers(self) -> None:
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    def _await_ok(self) -> None:
        assert self.ser, "Serial port not open"
        while (line := self.ser.readline().decode(errors="ignore").strip()):
            if "ok" in line:
                break

    # ── public API ───────────────────────────────────────────────────────────
    def send_command(self, gcode: str) -> None:
        assert self.is_connected and self.ser
        self.ser.write(f"{gcode}\n".encode())
        self._await_ok()
