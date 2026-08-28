#!/usr/bin/env python3
"""pv1800_monitor.py — MustPower PV1800 (SP1800, "18-series") status monitor.

Reads the inverter over RS-485 Modbus RTU (FC 0x03) and prints a decoded
status to the terminal. Uses pyserial for the serial port; the Modbus RTU
layer (frames, CRC) is implemented here — no pymodbus.

Hardware quirks handled (see PV1800 class):
  * the inverter answers only the first request after the port is opened,
    so every request uses a freshly opened port;
  * the inverter answers only sporadically (roughly 3 of 4 requests,
    worse after bursts); failed blocks are retried after a pause and the
    remaining blocks are still read;
  * the device drops the last register of each block (byte count field
    still says the full count); the parser accepts the shorter frame.

Protocol reference: doc/PV1800_protocol.md
  * RS-485, 19200 baud, 8 data bits, no parity, 1 stop bit
  * Modbus slave address 4
  * Blocks polled: 10001(8) charger id, 10103(10) charger settings,
    15201(21) charger status, 20000(17) inverter id, 20101(43) inverter
    settings, 25201(79) inverter status

Usage:
    python3 pv1800_monitor.py                  # single snapshot
    python3 pv1800_monitor.py --watch          # refresh every 2 s
    python3 pv1800_monitor.py --port /dev/ttyUSB1 --slave 4
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import serial

# ---------------------------------------------------------------------------
# Protocol constants (see doc/PV1800_protocol.md)
# ---------------------------------------------------------------------------

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 19200
DEFAULT_SLAVE = 4

# (name, start register, count) — the six blocks SolarPowerMonitor polls
BLOCKS = (
    ("charger_id", 10001, 8),
    ("charger_set", 10103, 10),
    ("charger_st", 15201, 21),
    ("inv_id", 20000, 17),
    ("inv_set", 20101, 43),
    ("inv_st", 25201, 79),
)

# Register addresses used for display (doc §6)
A = {
    # inverter identification
    "model_h": 20000, "model_l": 20001,
    "hw": 20004, "sw": 20005, "proto": 20006,
    # inverter status
    "work": 25201, "vac_class": 25202, "rated_va": 25203,
    "batt_v": 25205, "inv_v": 25206, "grid_v": 25207, "bus_v": 25208,
    "inv_i": 25210, "grid_i": 25211, "load_i": 25212,
    "p_inv": 25213, "p_grid": 25214, "p_load": 25215, "load_pct": 25216,
    "s_inv": 25217, "s_grid": 25218, "s_load": 25219,
    "f_inv": 25225, "f_grid": 25226,
    "t_ac": 25233, "t_tr": 25234, "t_dc": 25235,
    "r_inv": 25237, "r_grid": 25238, "r_load": 25239,
    "r_n": 25240, "r_dc": 25241, "r_earth": 25242,
    "err1": 25261, "err2": 25262, "err3": 25263,
    "warn1": 25265, "warn2": 25266,
    "batt_p": 25273, "batt_i": 25274, "batt_class": 25275, "rated_w": 25277,
    # charger status
    "c_work": 15201, "c_mppt": 15202, "c_charge": 15203,
    "pv_v": 15205, "c_batt_v": 15206, "c_i": 15207, "c_p": 15208,
    "c_t": 15209, "c_r_batt": 15211, "c_r_pv": 15212,
    "c_err": 15213, "c_warn": 15214, "c_batt_class": 15215, "c_rated_i": 15216,
    # charger settings
    "s_float": 10103, "s_absorb": 10104, "s_max_i": 10108,
    "s_batt_type": 10110, "s_batt_ah": 10111,
    # inverter settings
    "s_offgrid": 20101, "s_v_set": 20102, "s_f_set": 20103,
    "s_mode": 20109, "s_grid_prot": 20111, "s_max_disch_i": 20113,
    "s_stop_disch_v": 20118, "s_stop_ch_v": 20119, "s_grid_ch_i": 20125,
    "s_batt_low": 20127, "s_batt_high": 20128, "s_max_comb_i": 20132,
    "s_priority": 20143,
}

# Lifetime energy counters: (label, high word, low word), kWh = (H*1000+L)*0.1
ENERGY_PAIRS = (
    ("PV", 15217, 15218),
    ("grid charge", 25245, 25246),
    ("battery discharge", 25247, 25248),
    ("grid buy", 25249, 25250),
    ("grid sell", 25251, 25252),
    ("load", 25253, 25254),
    ("self-use", 25255, 25256),
    ("PV sell", 25257, 25258),
)

RELAYS = (
    ("inverter", "r_inv"), ("grid", "r_grid"), ("load", "r_load"),
    ("n-line", "r_n"), ("dc", "r_dc"), ("earth", "r_earth"),
)

WORK_STATE = {0: "PowerOn", 1: "SelfTest", 2: "OffGrid", 3: "GridTie",
              4: "ByPass", 5: "Stop", 6: "GridCharging"}
MPPT_STATE = {0: "Stop", 1: "MPPT", 2: "CurrentLimit"}
CHARGE_STATE = {0: "Stop", 1: "Absorb", 2: "Float"}
CHR_WORK = {0: "Initialization", 1: "Selftest", 2: "Work", 3: "Stop"}
BATTERY_TYPE = {1: "Defined", 2: "Lithium", 3: "SEALED_LEAD",
                4: "AGM", 5: "GEL", 6: "FLOODED"}
ENERGY_MODE = {1: "SBU", 2: "SUB", 3: "UTI", 4: "SOL"}
GRID_PROTECT = {0: "VDE4105", 1: "UPS", 2: "Home", 3: "GEN"}
CHARGE_PRIORITY = {0: "Solar first", 2: "Solar+Utility", 3: "Only solar"}

# Error / warning bit tables (doc §7). None = bit exists but no text defined.
INV_ERR1 = (
    "Fan locked (inverter off)", "Transformer over temperature",
    "Battery voltage too high", "Battery voltage too low",
    "Output short circuit", "Inverter output voltage high",
    "Overload time out", "Bus voltage too high",
    "Bus soft start failed", "Main relay failed",
    "Output voltage sensor error", "Grid voltage sensor error",
    "Output current sensor error", "Grid current sensor error",
    "Load current sensor error", "Grid over current error",
)
INV_ERR2 = (
    "Inverter radiator over temperature",
    "Solar charger battery voltage class error",
    "Solar charger current sensor error",
    "Solar charger current uncontrollable",
    "Inverter grid voltage low", "Inverter grid voltage high",
    "Inverter grid under frequency", "Inverter grid over frequency",
    "Inverter over current protection", "Bus voltage too low",
    "Inverter soft start failed", "Over DC voltage in AC output",
    "Battery connection open", "Control current sensor error",
    "Output voltage too low", None,
)
INV_WARN1 = (
    "Fan locked (inverter on)", "Fan2 locked (inverter on)",
    "Battery over-charged", "Low battery", "Overload",
    "Output power derating", "Solar charger stop: low battery",
    "Solar charger stop: high PV voltage",
    "Solar charger stop: overload", "Solar charger over temperature",
    "PV charger communication error",
)
CHR_ERR1 = (
    "Hardware protection", "Over current", "Current sensor error",
    "Over temperature", "PV voltage too high", None,
    "Battery voltage too high", "Battery voltage too low",
    "Current uncontrollable", "Parameter error",
)
CHR_WARN1 = ("Fan error",)


# ---------------------------------------------------------------------------
# Modbus RTU (FC 0x03) — minimal implementation, no pymodbus
# ---------------------------------------------------------------------------

def crc16(data: bytes) -> int:
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class ModbusError(Exception):
    pass


class NoResponse(ModbusError):
    pass


class ModbusException(ModbusError):
    CODES = {1: "IllegalFunction", 2: "IllegalDataAddress",
             3: "IllegalDataValue", 4: "SlaveDeviceFailure"}

    def __init__(self, code: int):
        super().__init__(self.CODES.get(code, f"exception code {code}"))


class PV1800:
    """PV1800 Modbus RTU client (FC 0x03) over RS-485 via pyserial.

    Hardware quirks observed on the connected unit (CH340 USB-RS485 adapter):

    1. The inverter answers only the FIRST Modbus request sent shortly after
       the serial port is opened; further requests on the same open port get
       no response.  Every request is therefore made on a freshly opened
       port (open -> settle -> send -> read -> close).
    2. The inverter answers only sporadically (observed: ~3 of 4 requests
       succeed, and rapid bursts are followed by longer silence).  Failed
       blocks are therefore retried after a pause (retry_pause).
    3. The device reports the full byte count in the response header but
       transmits only count-1 registers (the last register of each block is
       dropped); the CRC is computed over the truncated frame.  The parser
       accepts the shorter data section.
    """

    def __init__(self, device: str, baud: int, slave: int,
                 settle: float = 0.1, timeout: float = 0.6,
                 retries: int = 3, retry_pause: float = 1.5):
        self.device = device
        self.baud = baud
        self.slave = slave
        self.settle = settle      # pause after open before the first request
        self.timeout = timeout    # max wait for a response
        self.retries = retries
        self.retry_pause = retry_pause  # pause between retry attempts
        self.ser: serial.Serial | None = None

    # -- port lifecycle -------------------------------------------------------

    def _open(self):
        self.ser = serial.Serial(
            port=self.device,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,         # per-read; response loop below manages waits
            write_timeout=1.0,
            xonxoff=False,
            dsrdtr=False,
            rtscts=False,
        )
        self.ser.reset_input_buffer()
        time.sleep(self.settle)

    def _close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None

    # -- low level ------------------------------------------------------------

    def _read_response(self) -> bytes:
        """Collect one response frame: stop after a quiet gap or timeout."""
        assert self.ser is not None
        buf = b""
        deadline = time.monotonic() + self.timeout
        last = time.monotonic()
        while time.monotonic() < deadline:
            c = self.ser.read(1)
            if c:
                buf += c
                last = time.monotonic()
            elif buf and time.monotonic() - last > 0.05:  # frame done
                break
        return buf

    def _parse(self, buf: bytes, count: int):
        """Parse a response frame; return {offset: value} or None if invalid."""
        if len(buf) < 5:
            return None
        if buf[0] != self.slave:
            return None  # bus noise / not our frame
        if buf[1] == 0x83:
            raise ModbusException(buf[2])
        if buf[1] != 0x03:
            return None
        if len(buf) < 7:
            return None
        payload, crc = buf[:-2], buf[-2:]
        if crc16(payload) != int.from_bytes(crc, "little"):
            return None
        claimed = buf[2] // 2                    # registers per header
        actual = (len(buf) - 5) // 2             # registers really sent
        n = min(claimed, actual)                 # device may drop the last one
        if n <= 0:
            return None
        vals = struct.unpack(f">{n}H", buf[3:3 + 2 * n])
        return {i: v for i, v in enumerate(vals)}

    # -- public API ------------------------------------------------------------

    def read_block(self, start: int, count: int) -> dict[int, int]:
        """Read one register block on a freshly opened port.

        Returns {register offset within block: value}.
        """
        body = bytes([self.slave, 0x03,
                      (start >> 8) & 0xFF, start & 0xFF,
                      (count >> 8) & 0xFF, count & 0xFF])
        frame = body + struct.pack("<H", crc16(body))  # CRC low byte first
        last_err = NoResponse(f"no response from slave {self.slave}")
        buf = b""
        for attempt in range(self.retries):
            if attempt:
                time.sleep(self.retry_pause)
            try:
                self._open()
                self.ser.write(frame)
                buf = self._read_response()
            except OSError as e:
                last_err = e
            finally:
                self._close()
            parsed = self._parse(buf, count)
            if parsed is not None:
                return parsed
            last_err = NoResponse(
                f"no valid response from slave {self.slave} "
                f"({len(buf) if buf else 0} raw bytes)")
        raise last_err


# ---------------------------------------------------------------------------
# Decoding helpers
# ---------------------------------------------------------------------------

# Blocks refreshed on every watch cycle (SolarPowerMonitor 'alwaysRead').
STATUS_BLOCKS = (("charger_st", 15201, 21), ("inv_st", 25201, 79))


def read_all(pv: PV1800, full: bool = True,
             pause: float = 0.5) -> tuple[dict[int, int], dict[str, str]]:
    """Read blocks, return ({absolute register address: value}, {block: error}).

    full=True reads all six blocks (identification + settings + status);
    full=False reads only the two status blocks.  A block that fails after
    all retries is reported in the error dict; the other blocks are still
    read (the inverter only answers sporadically — see PV1800 docstring).
    """
    blocks = BLOCKS if full else STATUS_BLOCKS
    regs: dict[int, int] = {}
    errors: dict[str, str] = {}
    for i, (_name, start, count) in enumerate(blocks):
        if i:
            time.sleep(pause)  # let the bus settle between opens
        try:
            for off, val in pv.read_block(start, count).items():
                regs[start + off] = val
        except ModbusError as e:
            errors[_name] = str(e)
    return regs, errors


def fmt_edition(n: int) -> str:
    """10414 -> '1.04.14' (group digits in pairs from the right)."""
    s, parts = str(n), []
    while s:
        parts.append(s[-2:])
        s = s[:-2]
    return ".".join(reversed(parts))


def model_name(regs: dict[int, int]) -> str:
    mh, ml, ed = regs.get(20000), regs.get(20001), regs.get(20006)
    if mh is None or ml is None:
        return "PV1800 (unidentified)"
    if ed is not None and ed >= 10413:
        return chr((mh >> 8) & 0xFF) + chr(mh & 0xFF) + str(ml)
    return f"PV {ml}"


def energy_kwh(regs: dict[int, int], hi: int, lo: int) -> float:
    return (regs.get(hi, 0) * 1000 + regs.get(lo, 0)) * 0.1


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def _fmt_val(v, scale: float = 1.0, nd: int = 1, suffix: str = "") -> str:
    return "  --" if v is None else f"{v * scale:.{nd}f}{suffix}"


def _fmt_signed(v, scale: float = 1.0, nd: int = 1, suffix: str = "") -> str:
    if v is None:
        return "  --"
    if v >= 32768:
        v -= 65536
    return f"{v * scale:+.{nd}f}{suffix}"


def _state(v, table: dict) -> str:
    if v is None:
        return "--"
    return table.get(v, f"unknown({v})")


def _onoff(v) -> str:
    if v is None:
        return "?"
    return "ON" if v else "OFF"


def _bits(v, table) -> list[str]:
    out = []
    for bit in range(16):
        if v and (v >> bit) & 1:
            name = table[bit] if bit < len(table) and table[bit] else None
            out.append(f"bit {bit}: {name or 'undefined bit'}")
    return out


def render(regs: dict[int, int]) -> str:
    def g(key):
        return regs.get(A[key])

    lines: list[str] = []
    add = lines.append

    # --- header -------------------------------------------------------------
    add(f"MustPower {model_name(regs)}  —  status")
    add("=" * 60)
    add(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    proto = g("proto")
    add(f"Firmware: HW {fmt_edition(g('hw') or 0)}, SW {fmt_edition(g('sw') or 0)}, "
        f"protocol {fmt_edition(proto) if proto is not None else '--'}")
    add("")

    # --- inverter -----------------------------------------------------------
    add("[Inverter]")
    add(f"  Work state:   {_state(g('work'), WORK_STATE)}")
    add(f"  Output:       {_fmt_val(g('inv_v'), 0.1)} V / "
        f"{_fmt_val(g('f_inv'), 0.01, 2)} Hz   "
        f"P={_fmt_val(g('p_inv'), 1, 0)} W  S={_fmt_val(g('s_inv'), 1, 0)} VA  "
        f"load {g('load_pct') if g('load_pct') is not None else '--'}%")
    add(f"  Grid:         {_fmt_val(g('grid_v'), 0.1)} V / "
        f"{_fmt_val(g('f_grid'), 0.01, 2)} Hz   "
        f"P={_fmt_signed(g('p_grid'), 1, 0)} W  S={_fmt_signed(g('s_grid'), 1, 0)} VA")
    add(f"  DC bus:       {_fmt_val(g('bus_v'), 0.1)} V")
    batt_i = g("batt_i")
    batt_i_s = (batt_i - 65536) if (batt_i is not None and batt_i >= 32768) else batt_i
    if batt_i_s is None:
        batt_dir = "--"
    elif batt_i_s > 0:
        batt_dir = "discharge"
    elif batt_i_s < 0:
        batt_dir = "charge"
    else:
        batt_dir = "idle"
    add(f"  Battery:      {_fmt_val(g('batt_v'), 0.1)} V  "
        f"{_fmt_signed(batt_i_s, 1, 0)} A ({batt_dir})  "
        f"{_fmt_val(g('batt_p'), 1, 0)} W   "
        f"({g('batt_class') if g('batt_class') is not None else '--'} V class)")
    add(f"  Temperatures: AC heatsink {g('t_ac') if g('t_ac') is not None else '--'} C, "
        f"DC heatsink {g('t_dc') if g('t_dc') is not None else '--'} C, "
        f"transformer {g('t_tr') if g('t_tr') is not None else '--'} C")
    rel = "  ".join(f"{name} {_onoff(g(key))}" for name, key in RELAYS)
    add(f"  Relays:       {rel}")
    add("")

    # --- charger -------------------------------------------------------------
    add("[Charger / MPPT]")
    add(f"  Work state:   {_state(g('c_work'), CHR_WORK)}   "
        f"MPPT: {_state(g('c_mppt'), MPPT_STATE)}   "
        f"charging: {_state(g('c_charge'), CHARGE_STATE)}")
    add(f"  PV input:     {_fmt_val(g('pv_v'), 0.1)} V   "
        f"{_fmt_val(g('c_i'), 0.1)} A   {_fmt_val(g('c_p'), 1, 0)} W")
    add(f"  Battery:      {_fmt_val(g('c_batt_v'), 0.1)} V   "
        f"rated current {_fmt_val(g('c_rated_i'), 0.1)} A")
    add(f"  Heatsink:     {g('c_t') if g('c_t') is not None else '--'} C   "
        f"relays: batt {_onoff(g('c_r_batt'))}, pv {_onoff(g('c_r_pv'))}")
    add("")

    # --- settings -------------------------------------------------------------
    add("[Settings]")
    add(f"  Output setpoint: {_fmt_val(g('s_v_set'), 0.1)} V / "
        f"{_fmt_val(g('s_f_set'), 0.01, 2)} Hz   "
        f"off-grid enable: {g('s_offgrid') if g('s_offgrid') is not None else '--'}")
    add(f"  Energy mode:     {_state(g('s_mode'), ENERGY_MODE)}   "
        f"grid protection: {_state(g('s_grid_prot'), GRID_PROTECT)}   "
        f"priority: {_state(g('s_priority'), CHARGE_PRIORITY)}")
    bt = g("s_batt_type")
    bt_s = BATTERY_TYPE.get(bt, f"unknown({bt})") if bt is not None else "--"
    add(f"  Battery:         {bt_s} "
        f"{g('s_batt_ah') if g('s_batt_ah') is not None else '--'} Ah, "
        f"float {_fmt_val(g('s_float'), 0.1)} V, absorb {_fmt_val(g('s_absorb'), 0.1)} V, "
        f"max charge {_fmt_val(g('s_max_i'), 0.1)} A")
    add(f"  Charge limits:   stop discharge {_fmt_val(g('s_stop_disch_v'), 0.1)} V, "
        f"stop charge {_fmt_val(g('s_stop_ch_v'), 0.1)} V, "
        f"low {_fmt_val(g('s_batt_low'), 0.1)} V, high {_fmt_val(g('s_batt_high'), 0.1)} V")
    add(f"  Max currents:    discharge {_fmt_val(g('s_max_disch_i'), 0.1)} A, "
        f"grid charge {_fmt_val(g('s_grid_ch_i'), 0.1)} A, "
        f"combined {_fmt_val(g('s_max_comb_i'), 0.1)} A")
    add("")

    # --- energy ---------------------------------------------------------------
    add("[Energy (lifetime)]")
    parts = [f"{label} {energy_kwh(regs, hi, lo):.1f} kWh"
             for label, hi, lo in ENERGY_PAIRS]
    for i in range(0, len(parts), 3):
        add("  " + "  ".join(parts[i:i + 3]))
    add("")

    # --- errors / warnings ------------------------------------------------------
    add("[Errors / Warnings]")
    problems: list[str] = []
    for label, key, table in (
            ("Inverter error", "err1", INV_ERR1),
            ("Inverter error", "err2", INV_ERR2),
            ("Charger error", "c_err", CHR_ERR1)):
        v = g(key)
        for b in _bits(v, table):
            problems.append(f"  {label} {b}")
    v = g("err3")
    if v:
        problems.append(f"  Inverter error3: raw 0x{v:04X} (no text table)")
    for label, key, table in (
            ("Inverter warning", "warn1", INV_WARN1),
            ("Charger warning", "c_warn", CHR_WARN1)):
        v = g(key)
        for b in _bits(v, table):
            problems.append(f"  {label} {b}")
    v = g("warn2")
    if v:
        problems.append(f"  Inverter warning2: raw 0x{v:04X} (no text table)")
    if problems:
        lines.extend(problems)
    else:
        add("  none active")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="MustPower PV1800 (18-series) RS-485 Modbus RTU status monitor")
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"serial device (default {DEFAULT_PORT})")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help=f"baud rate (default {DEFAULT_BAUD})")
    ap.add_argument("--slave", type=int, default=DEFAULT_SLAVE,
                    help=f"Modbus slave address (default {DEFAULT_SLAVE})")
    ap.add_argument("--timeout", type=float, default=0.6,
                    help="response timeout in seconds (default 0.6)")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="refresh interval in seconds for --watch (default 2.0)")
    ap.add_argument("--retries", type=int, default=3,
                    help="attempts per register block (default 3)")
    ap.add_argument("--watch", action="store_true",
                    help="refresh continuously instead of a single snapshot")
    args = ap.parse_args()

    pv = PV1800(args.port, args.baud, args.slave, timeout=args.timeout,
                retries=args.retries)
    is_tty = sys.stdout.isatty()
    regs: dict[int, int] = {}
    last_good: str | None = None
    first_cycle = True
    exit_code = 0

    try:
        while True:
            try:
                # First cycle: full read (id + settings + status).  Later
                # watch cycles refresh only the two status blocks, as the
                # reference monitor does (doc §5).
                new, errors = read_all(pv, full=first_cycle)
                if not new:
                    raise ModbusError(
                        "; ".join(f"{k}: {v}" for k, v in errors.items())
                        or f"no response from slave {args.slave}")
                regs.update(new)
                last_good = render(regs)
                if errors:
                    last_good += "\n" + "\n".join(
                        f"  !! {name} read failed (stale value kept): {err}"
                        for name, err in errors.items())
                if is_tty and args.watch:
                    sys.stdout.write("\x1b[2J\x1b[H" + last_good + "\n")
                else:
                    print(last_good)
            except ModbusError as e:
                if not args.watch:
                    print(f"error: {e}", file=sys.stderr)
                    return 1
                msg = f"  !! communication error: {e} (keeping last snapshot)"
                if last_good is not None:
                    sys.stdout.write("\x1b[2J\x1b[H" + last_good + "\n" + msg + "\n")
                else:
                    print(msg)
            sys.stdout.flush()
            first_cycle = False
            if not args.watch:
                break
            try:
                time.sleep(args.interval)
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        pass
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
