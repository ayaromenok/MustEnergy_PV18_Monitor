# MustPower PV1800 (SP1800) Communication Protocol


## 1. Overview

| Item | Value |
|---|---|
| Device | MustPower PV1800 hybrid inverter / MPPT solar charger (12 V battery class, 1500 VA / 1500 W) |
| Product code in SolarPowerMonitor | `DeviceType.Sp1800` (enum `Domin.Mods.DeviceType`) |
| Protocol family | `ProtocolType.Ph18Series` (the "18-series" RS-485 protocol, shared with PH1800/EP1800) |
| Data model class | `Domin.Entities.Ph1800M` (persisted in EF table `Rs485Ms`) |
| Transport | RS-485, half-duplex, **Modbus RTU** |
| Modbus slave address | **4** (decimal) for the PV1800 |
| Baud rate | **19200** (8 data bits, no parity, 1 stop bit) |
| Protocol edition reported by the captured unit | `20006 = 10414` → formatted **"1.04.14"** |
| Machine type reported by the captured unit | `20000 = 0x5056` ("PV"), `20001 = 1800` → **"PV1800"** |

The device is a single Modbus slave that exposes two logical register "spaces" in one map:

* **Charger (MPPT) section** — registers `1xxxx`
* **Inverter section** — registers `2xxxx`

Each section is split into: identification (`…0000`), settings (`…0100`) and status (`…5200`) blocks.

> Naming note: the monitor's `DeviceType.Sp1800` entry uses exactly the same read blocks as `Ph1800`, and `Ph1800M.SystemSettingBit` contains PV1800-specific flags ("GridBuzzEnable (only use by PV1800)", "BuzzForbide (only use by PV1800)"). PV1800/SP1800 and PH1800 therefore share the `Ph18Series` protocol; the monitor distinguishes them only by the machine-type registers.

---

## 2. Physical / Link Layer

| Parameter | Value | Evidence |
|---|---|---|
| Interface | RS-485 (EF DbSet is literally named `Rs485Ms`); multi-drop bus, devices addressed by Modbus slave ID | `PmEntities` |
| Baud rate | 19200 | `CommParams` seed for `Ph18Series` (`PortBaudrate = 19200`); capture: `IOCTL_SERIAL_SET_BAUD_RATE = 0x4B00` |
| Data bits / parity / stop | 8 / None / 1 | `CommParams` defaults (`DataBit = 8`, `StopBits = 1`); capture: `IOCTL_SERIAL_SET_LINE_CONTROL = 00 00 08` |
| Handshake | none | capture: `SET_HANDFLOW` fOutX/fInX/fDtr/fRts control = 0 |
| CRC | Modbus CRC-16 (poly 0xA001, init 0xFFFF), transmitted **low byte first** | all 67 captured frames verified |
| Read timeout | host uses event-driven reads (`IOCTL_SERIAL_WAIT_ON_MASK`), total/interval read timeouts disabled (`0xFFFFFFFF / 0xFFFEFFFE / 0`) | capture |
| Inter-frame delay | host waits after each probe: `AfterScanTxWait = 300 ms` for Ph18Series (100 ms default for other protocols) | `PmEntities` `DefaultParamses` |

Only **Function Code 0x03 (Read Holding Registers)** was observed in the capture (30 requests to slave 4, 13 responses). No exception frames (`0x83`) occurred. Write frames are standard Modbus FC 0x06 / 0x10 (see §8) but were not exercised in this capture.

---

## 3. Frame Format (FC 0x03)

**Request (8 bytes):**

```
+--------+--------+--------+--------+--------+--------+--------+--------+
| Slave  |  FC    | Start  | Start  | Count  | Count  | CRC    | CRC    |
| addr   | = 0x03 | addr H | addr L | (H)    | (L)    | (L)    | (H)    |
+--------+--------+--------+--------+--------+--------+--------+--------+
```

**Response (5 + 2·N + 2 bytes):**

```
| Slave | 0x03 | Byte count = 2N | N × 16-bit registers (big-endian) | CRC-L | CRC-H |
```

### 3.1 Request examples (from capture)

| Hex | Slave | Start (dec) | Count | Purpose |
|---|---|---|---|---|
| `04 03 27 11 00 08 1E E8` | 4 | 10001 | 8 | charger identification |
| `04 03 27 77 00 0A 7F 36` | 4 | 10103 | 10 | charger settings |
| `04 03 3B 61 00 15 D8 AA` | 4 | 15201 | 21 | charger status |
| `04 03 4E 20 00 11 93 71` | 4 | 20000 | 17 | inverter identification |
| `04 03 4E 85 00 2B 03 41` | 4 | 20101 | 43 | inverter settings |
| `04 03 62 71 00 4F 4B C8` | 4 | 25201 | 79 | inverter status |

### 3.2 Response example (from capture, 15201 block, 21 registers)

```
04 03 2A 00 01 00 00 00 00 00 00 00 00 00 7D 00 00 00 00 00 2C 00 00
00 00 00 00 20 00 00 00 0C 02 58 00 00 00 02 00 00 00 00 00 00 02 6B
```

Decoded: charger work state = 1 (Selftest), battery voltage = 0x007D → 12.5 V,
radiator temp = 0x002C → 44 °C, battery-voltage grade = 12 V, rated current = 0x0258 → 60.0 A.

---

## 4. Device Scanning / Identification

SolarPowerMonitor does not know the device in advance. It cycles through the
protocol presets (order and parameters from `PmEntities.DefaultParamses`) and
sends a probe **FC 0x03 read of 1 register** (7 registers for the 18-series) at
each preset's scan address:

| Step | Baud | Slave ID | Probe frame | Protocol preset |
|---|---|---|---|---|
| 1 | 9600 | 1 | `01 03 27 10 00 01 8F 7B` (start 10000) | Pc1800 |
| 2 | 9600 | 5 | `05 03 4E 21 00 01 C2 AC` (start 20001) | Ph1000 |
| 3 | 9600 | 6 | `06 03 4E 21 00 01 C2 9F` (start 20001) | Ph5000 |
| 4 | 9600 | 10 | `0A 03 75 30 00 01 9F 72` (start 30000) | EPSeries |
| 5 | 19200 | 4 | `04 03 4E 20 00 07 12 BF` (start 20000, **7 regs**) | Ph18Series |

Observed in the capture: steps 1–4 get no answer; step 5 is answered by the
PV1800:

```
04 03 0E 50 56 07 08 FF FF FF FF 27 75 13 29 28 AE 4B C9
```

The host then classifies the device using registers 20000–20006:

* `ProtocalEditionNo (20006) >= 10413` → machine type = `ASCII(20000) + 20001`
  (here "PV" + 1800 = **PV1800**);
* otherwise → machine type = `"PV " + 20001`.

The scan loop (≈ 2 s per cycle, one probe per ~1 s) repeats until a known
device answers; after identification the port stays at 19200 and the data
polling phase begins.

---

## 5. Polling Sequence (data phase)

Register blocks polled for `DeviceType.Sp1800`,
all at slave ID 4):

| # | Start | Count | Block | `alwaysRead` |
|---|---|---|---|---|
| 1 | 10001 | 8 | charger identification | no |
| 2 | 10103 | 10 | charger settings | no |
| 3 | 15201 | 21 | charger status | **yes** |
| 4 | 20000 | 17 | inverter identification | no |
| 5 | 20101 | 43 | inverter settings | no |
| 6 | 25201 | 79 | inverter status | **yes** |

Observed timeline (capture):

```
13:42:58  scan probe 4@20000(7) answered  → identification
13:42:58  read 10001(8)                    ┐
13:42:59  read 10103(10)                   │ initial full read
13:42:59  read 15201(21)                   │ of all 6 blocks
13:42:59  read 20000(17)                   │ (~2 s)
13:43:00  read 20101(43)                   │
13:43:00  read 25201(79)                   ┘
13:43:01  read 15201(21)                   ┐
13:43:02  read 25201(79)                   ├ steady state: the two
13:43:03  read 15201(21)                   │ alwaysRead blocks,
13:43:04  read 25201(79)                   │ 1 s apart, 2 s cycle
...
```

So in steady state the monitor refreshes charger status and inverter status
once per second (alternating), i.e. a 2 s full refresh cycle. Non-`alwaysRead`
blocks (identification/settings) are read once after identification and after
user changes.

---

## 6. Register Map

Scales below are taken from the `[Modbus(address, scale, …)]` attributes in
`Ph1800M`. "Observed" values are from the capture.
`R` = read, `W` = writable (member of `Ph1800M.AffectAddress`, the set of
registers the monitor treats as user settings — see §8).

### 6.1 Inverter identification — 20000 … 20016 (17 regs)

| Addr | Name | Scale | R/W | Observed | Meaning |
|---|---|---:|---|---|---|
| 20000 | MachineTypeH | 1 | R | 20566 (0x5056) | ASCII pair, high word → `"PV"` |
| 20001 | MachineTypeL | 1 | R | 1800 | machine-type number → model = "PV1800" |
| 20002 | SerialNumberH | 1 | R | 65535 | serial number high word (0xFFFF = not set) |
| 20003 | SerialNumberL | 1 | R | 65535 | serial number low word |
| 20004 | HardwareNo | 1 | R | 10101 | hardware version → "10.101" |
| 20005 | SoftwareNo | 1 | R | 4905 | firmware version → "49.05" |
| 20006 | ProtocalEditionNo | 1 | R | 10414 | protocol edition → "1.04.14" |
| 20007 | — | 1 | R | 65535 | reserved |
| 20008 | — | 1 | R | 65535 | reserved |
| 20009 | BatteryVoltageC | 1 | R | 65535 | raw battery-voltage count (unused; display value comes from 25205) |
| 20010 | InverterVoltageC | 1 | R | 65535 | raw inverter-voltage count (unused) |
| 20011 | GridVoltageC | 1 | R | 65535 | raw grid-voltage count (unused) |
| 20012 | BusVoltageC | 1 | R | 65535 | raw DC-bus-voltage count (unused) |
| 20013 | ControlCurrentC | 1 | R | 65535 | raw control-current count (unused) |
| 20014 | InverterCurrentC | 1 | R | 65535 | raw inverter-current count (unused) |
| 20015 | GridCurrentC | 1 | R | 65535 | raw grid-current count (unused) |
| 20016 | LoadCurrentC | 1 | R | 65535 | raw load-current count (unused) |

Version formatting (`Helper.FormatEdition(value, 2)`): group digits in pairs
from the right, dot-separated: 10101 → `10.101`, 4905 → `49.05`,
10414 → `1.04.14`.

### 6.2 Inverter settings — 20101 … 20143 (43 regs)

| Addr | Name | Scale | R/W | Observed | Meaning / value set |
|---|---|---:|---|---|---|
| 20101 | InverterOffgridWorkEnable | 1 | R/W | 1 | off-grid (standalone) operation enable, 0/1 |
| 20102 | InverterOutputVoltageSet | 0.1 V | R/W | 2300 → 230.0 V | output voltage setpoint |
| 20103 | InverterOutputFrequencySet | 0.01 Hz | R/W | 5000 → 50.00 Hz | output frequency setpoint |
| 20104 | InverterSearchModeEnable | 1 | R/W | 0 | grid-search mode enable, 0/1 |
| 20105 | (unmapped) | 1 | R | 1 | — |
| 20106 | (unmapped) | 1 | R | 1 | — |
| 20107 | (unmapped) | 1 | R | 1 | — |
| 20108 | InverterDischargerToGridEnable | 1 | R/W | 0 | discharge-to-grid enable, 0/1 |
| 20109 | EnergyUseMode | 1 | R/W | 3 | 0=–, 1=SBU, 2=SUB, 3=UTI, 4=SOL |
| 20110 | (unmapped) | 1 | R | 0 | — |
| 20111 | GridProtectStandard | 1 | R/W | 2 | 0=VDE4105, 1=UPS, 2=Home, 3=GEN |
| 20112 | SolarUseAim | 1 | R/W | 0 | 0=LBU, 1=BLU |
| 20113 | InverterMaxDischargerCurrent | 0.1 A | R/W | 66 → 6.6 A | max inverter discharge current |
| 20114 | (unmapped) | 1 | R | 66 | — |
| 20115 | (unmapped) | 1 | R | 0 | — |
| 20116 | (unmapped) | 1 | R | 0 | — |
| 20117 | (unmapped) | 1 | R | 113 | — |
| 20118 | BatteryStopDischargingVoltage | 0.1 V | R/W | 115 → 11.5 V | |
| 20119 | BatteryStopChargingVoltage | 0.1 V | R/W | 142 → 14.2 V | |
| 20120 | (unmapped) | 1 | R | 0 | — |
| 20121 | (unmapped) | 1 | R | 125 | — |
| 20122 | (unmapped) | 1 | R | 135 | — |
| 20123 | (unmapped) | 1 | R | 135 | — |
| 20124 | (unmapped) | 1 | R | 12 | — |
| 20125 | GridMaxChargerCurrentSet | 0.1 A | R/W | 200 → 20.0 A | |
| 20126 | (unmapped) | 1 | R | 200 | — |
| 20127 | BatteryLowVoltage | 0.1 V | R/W | 110 → 11.0 V | |
| 20128 | BatteryHighVoltage | 0.1 V | R/W | 150 → 15.0 V | |
| 20129 | (unmapped) | 1 | R | 2 | — |
| 20130 | (unmapped) | 1 | R | 200 | — |
| 20131 | (unmapped) | 1 | R | 115 | — |
| 20132 | MaxCombineChargerCurrent | 0.1 A | R/W | 600 → 60.0 A | max combined (PV+grid) charge current |
| 20133–20141 | (unmapped) | 1 | R | 0 | — |
| 20142 | SystemSetting | 1 | R/W | 114 | 16-bit feature mask (see §6.7) |
| 20143 | ChargerSourcePriority | 1 | R/W | 2 | 0=Solar first, 2=Solar and Utility (default), 3=Only Solar |

### 6.3 Inverter status — 25201 … 25279 (79 regs)

| Addr | Name | Scale | Observed | Meaning |
|---|---|---:|---|---|
| 25201 | WorkStateNo | 1 | 2 | 0=PowerOn, 1=SelfTest, 2=OffGrid, 3=Grid-Tie, 4=ByPass, 5=Stop, 6=Grid charging |
| 25202 | AcVoltageGrade | 1 V | 230 | output AC voltage class |
| 25203 | RatedPower | 1 VA | 1500 | rated apparent power |
| 25204 | (unmapped) | 1 | 0 | — |
| 25205 | BatteryVoltage | 0.1 V | 126 → 12.6 V | |
| 25206 | InverterVoltage | 0.1 V | 2284–2300 → 228.4–230.0 V | |
| 25207 | GridVoltage | 0.1 V | 0 | |
| 25208 | BusVoltage | 0.1 V | 4630–4631 → 463.0–463.1 V | internal DC bus |
| 25209 | ControlCurrent | 0.1 A | 4 → 0.4 A | |
| 25210 | InverterCurrent | 0.1 A | 0 | |
| 25211 | GridCurrent | 0.1 A | 0 | |
| 25212 | LoadCurrent | 0.1 A | 0 | |
| 25213 | PInverter | 1 W | 0 | inverter active power |
| 25214 | PGrid | 1 W | 0 | grid active power (signed) |
| 25215 | PLoad | 1 W | 0 | load active power |
| 25216 | LoadPercent | 1 % | 0 | load percentage of rating |
| 25217 | SInverter | 1 VA | 100 | inverter apparent power |
| 25218 | SGrid | 1 VA | 0 | grid apparent power |
| 25219 | Sload | 1 VA | 0 | load apparent power |
| 25220 | (unmapped) | 1 | 0 | — |
| 25221 | Qinverter | 1 var | 97–99 | inverter reactive power |
| 25222 | Qgrid | 1 var | 0 | |
| 25223 | Qload | 1 var | 100–102 | load reactive power |
| 25224 | (unmapped) | 1 | 0 | — |
| 25225 | InverterFrequency | 0.01 Hz | 4999–5000 → 49.99–50.00 Hz | |
| 25226 | GridFrequency | 0.01 Hz | 0 | |
| 25227 | (unmapped) | 1 | 0 | — |
| 25228 | (unmapped) | 1 | 0 | — |
| 25229 | InverterMaxNumber | 1 | 0 | max. inverters in parallel |
| 25230 | CombineType | 1 | 0 | parallel-combine type |
| 25231 | InverterNumber | 1 | 0 | this inverter's index in parallel group |
| 25232 | (unmapped) | 1 | 0 | — |
| 25233 | AcRadiatorTemp | 1 °C | 60 | |
| 25234 | TransformerTemp | 1 °C | 0 | |
| 25235 | DcRadiatorTemp | 1 °C | 76 | |
| 25236 | (unmapped) | 1 | 0 | — |
| 25237 | InverterRelayStateNo | 1 | 1 | 0=Disconnect, 1=Connect |
| 25238 | GridRelayStateNo | 1 | 0 | 0=Disconnect, 1=Connect |
| 25239 | LoadRelayStateNo | 1 | 1 | 0=Disconnect, 1=Connect |
| 25240 | NLineRelayStateNo | 1 | 0 | 0=Disconnect, 1=Connect |
| 25241 | DcRelayStateNo | 1 | 1 | 0=Disconnect, 1=Connect |
| 25242 | EarthRelayStateNo | 1 | 0 | 0=Disconnect, 1=Connect |
| 25243 | (unmapped) | 1 | 0 | — |
| 25244 | (unmapped) | 1 | 0 | — |
| 25245/25246 | AccumulatedChargerPower H/L | 1 | 0 / 0 | energy from grid charging, kWh = (H·1000 + L) × 0.1 |
| 25247/25248 | AccumulatedDischargerPower H/L | 1 | 0 / 14 | battery discharge energy → 1.4 kWh |
| 25249/25250* | AccumulatedBuyPower H/L | 1 | 0 / 0 | energy bought from grid (*monitor maps L to 25259 — see note) |
| 25251/25252 | AccumulatedSellPower H/L | 1 | 0 / 0 | energy sold to grid |
| 25253/25254 | AccumulatedLoadPower H/L | 1 | 0 / 156 | load energy → 15.6 kWh |
| 25255/25256 | AccumulatedSelfusePower H/L | 1 | 0 / 14 | self-consumed energy → 1.4 kWh |
| 25257/25258 | AccumulatedPvsellPower H/L | 1 | 0 / 0 | PV energy sold |
| 25259/25260 | AccumulatedGridChargerPower H/L | 1 | 0 / 0 | grid-charge energy |
| 25261 | Error1 | 1 (bitmask) | 0 | inverter errors, bits 0–15 (§7.1) |
| 25262 | Error2 | 1 (bitmask) | 0 | inverter errors, bits 16–31 (§7.2) |
| 25263 | Error3 | 1 (bitmask) | 0 | inverter errors, bits 32–47 (no text table in monitor) |
| 25264 | (unmapped) | 1 | 0 | — |
| 25265 | Warning1 | 1 (bitmask) | 0 | inverter warnings, bits 0–15 (§7.3) |
| 25266 | Warning2 | 1 (bitmask) | 0 | inverter warnings, bits 16–31 |
| 25267 | (unmapped) | 1 | 0 | — |
| 25268 | (unmapped) | 1 | 0 | — |
| 25269 | (unmapped) | 1 | 65535 | duplicate of reserved word |
| 25270 | (unmapped) | 1 | 65535 | duplicate of reserved word |
| 25271 | (unmapped) | 1 | 10101 | duplicate of hardware version (20004) |
| 25272 | (unmapped) | 1 | 4905 | duplicate of software version (20005) |
| 25273 | BattPower | 1 W | 20 | battery power |
| 25274 | BattCurrent | 1 A | 1 | battery current (signed; + = discharge in monitor logic) |
| 25275 | BattVoltageGrade | 1 V | 12 | battery voltage class (12 V system) |
| 25276 | (unmapped) | 1 | 0 | — |
| 25277 | RatedPowerW | 1 W | 1500 | rated power in watts |
| 25278 | CommunicationProtocalEdition | 1 | 10414 | duplicate of 20006 |
| 25279 | ArrowFlag | 1 | 166 | UI hint (arrow direction on LCD) |

> **Note (monitor quirk):** in `Ph1800M` both `AccumulatedBuyPowerL` and
> `AccumulatedGridChargerPowerH` are mapped to register 25259. The physical
> layout implied by the frame is 25249/25250 = buy H/L and 25259/25260 =
> grid-charger H/L; the monitor's mapping of the buy-energy low word to 25259
> is a software bug and should not be replicated.

Energy registers are 32-bit values split big-endian (H word first) with a
resolution of 0.1 kWh: `kWh = (H × 1000 + L) × 0.1`.

### 6.4 Charger identification — 10001 … 10008 (8 regs)

| Addr | Name | Scale | Observed | Meaning |
|---|---|---:|---|---|
| 10001 | ChrMachineType | 1 | 1800 | charger machine type (1800) |
| 10002 | ChrSerialNumberH | 1 | 0 | charger serial H |
| 10003 | ChrSerialNumberL | 1 | 0 | charger serial L |
| 10004 | ChrHardwareNo | 1 | 10102 | charger hardware version → "10.102" |
| 10005 | ChrSoftwareNo | 1 | 805 | charger firmware version → "8.05" |
| 10006 | PvVoltageC | 1 | 0 | raw PV voltage count (unused) |
| 10007 | ChrBatteryVoltageC | 1 | 0 | raw battery voltage count (unused) |
| 10008 | ChargerCurrentC | 1 | 0 | raw charger current count (unused) |

### 6.5 Charger settings — 10103 … 10112 (10 regs)

| Addr | Name | Scale | R/W | Observed | Meaning |
|---|---|---:|---|---|---|
| 10103 | FloatVoltage | 0.1 V | R/W | 135 → 13.5 V | |
| 10104 | AbsorptionVoltage | 0.1 V | R/W | 135 → 13.5 V | |
| 10105 | ChrBatteryLowVoltage | 0.1 V | R/W | 85 → 8.5 V | |
| 10106 | (unmapped in monitor) | 1 | R | 85 | (likely battery-low/secondary threshold) |
| 10107 | (unmapped in monitor) | 1 | R | 150 | |
| 10108 | MaxChargerCurrent | 0.1 A | R/W | 600 → 60.0 A | |
| 10109 | (unmapped in monitor) | 1 | R | 100 | |
| 10110 | BatteryType | 1 | R/W | 2 | 0=–, 1=Use defined battery, 2=Lithium, 3=SEALED_LEAD, 4=AGM, 5=GEL, 6=FLOODED |
| 10111 | BatteryAh | 1 Ah | R | 200 | battery capacity |
| 10112 | RemoveTheAccumulatedData | 1 | W | 0 | command: write to clear accumulated energy data |

Extended charger-settings registers
(not read by the monitor for SP1800, part of the wider 18-series charger map):
10101 ChargerWorkEnable, 10113 BatteryVoltageGrade, 10116 CvChargingMaxTime,
10117 BtsTemperatureCompensationRatio (×0.1), 10118 BatteryEqualizationEnable,
10119 BatteryEqualizationVoltage (×0.1), 10120 MaxCurrentOfBatteryEqualization
(×0.1), 10121 BatteryEqualizedTime, 10122 BatteryEqualizedTimeout,
10123 EqualizationInterval, 10124 EqualizationActivedImmediately (command),
10125 SystemSetting, 10126 ResetTheParameter (command).

### 6.6 Charger status — 15201 … 15221 (21 regs)

| Addr | Name | Scale | Observed | Meaning |
|---|---|---:|---|---|
| 15201 | ChrWorkstateNo | 1 | 1 | 0=Initialization, 1=Selftest, 2=Work, 3=Stop |
| 15202 | MpptStateNo | 1 | 0 | 0=Stop, 1=MPPT, 2=Current limiting |
| 15203 | ChargingStateNo | 1 | 0 | 0=Stop, 1=Absorb charge, 2=Float charge |
| 15204 | (unmapped) | 1 | 0 | — |
| 15205 | PvVoltage | 0.1 V | 0 | PV array voltage |
| 15206 | ChrBatteryVoltage | 0.1 V | 125 → 12.5 V | |
| 15207 | ChargerCurrent | 0.1 A | 0 | |
| 15208 | ChargerPower | 1 W | 0 | |
| 15209 | RadiatorTemp | 1 °C | 44 | charger heatsink temperature |
| 15210 | ExternalTemp | 1 °C | 0 | external temperature sensor |
| 15211 | BatteryRelayNo | 1 | 0 | 0=Disconnect, 1=Connect |
| 15212 | PvRelayNo | 1 | 0 | 0=Disconnect, 1=Connect |
| 15213 | ChrError1 | 1 (bitmask) | 32 | charger errors, bits 0–15 (§7.4) — bit 5 set (no text defined in monitor) |
| 15214 | ChrWarning1 | 1 (bitmask) | 0 | charger warnings, bits 0–15 (§7.5) |
| 15215 | BattVolGrade | 1 V | 12 | battery voltage class |
| 15216 | RatedCurrent | 0.1 A | 600 → 60.0 A | charger rated current |
| 15217/15218 | AccumulatedPvPower H/L | 1 | 0 / 2 | PV energy → 0.2 kWh |
| 15219 | AccumulatedDay | 1 | 0 | PV charging time: days |
| 15220 | AccumulatedHour | 1 | 0 | PV charging time: hours |
| 15221 | AccumulatedMinute | 1 | 0 | PV charging time: minutes |

### 6.7 SystemSetting bit field (register 20142)

16-bit mask; bit names from `Ph1800M.SystemSettingBit`:

| Bit | Name |
|---:|---|
| 0 | OverLoadRestartForbid |
| 1 | OverTempRestartForbid |
| 2 | OverLoadBypassForbid |
| 3 | AutoTurnPageFlagForbid |
| 4 | GridBuzzEnable (PV1800 only) |
| 5 | BuzzForbide (PV1800 only) |
| 6 | LcdLightEnable |
| 7 | RecordFaultForbid |
| 8–15 | reserved |

Observed value 114 = 0b0000_0111_0010 → bits 1, 4, 5, 6 set
(OverTempRestartForbid, GridBuzzEnable, BuzzForbide, LcdLightEnable).

---

## 7. Error / Warning Bit Fields

Bit 0 = LSB. A bit set = condition active (reported since last clear).

### 7.1 Inverter Error1 (25261), bits 0–15

0 Fan is locked when inverter is off · 1 Inverter transformer over temperature ·
2 Battery voltage is too high · 3 Battery voltage is too low ·
4 Output short circuited · 5 Inverter output voltage is high ·
6 Overload time out · 7 Inverter bus voltage is too high ·
8 Bus soft start failed · 9 Main relay failed ·
10 Inverter output voltage sensor error · 11 Inverter grid voltage sensor error ·
12 Inverter output current sensor error · 13 Inverter grid current sensor error ·
14 Inverter load current sensor error · 15 Inverter grid over current error

### 7.2 Inverter Error2 (25262), bits 0–15

0 Inverter radiator over temperature · 1 Solar charger battery voltage class error ·
2 Solar charger current sensor error · 3 Solar charger current is uncontrollable ·
4 Inverter grid voltage is low · 5 Inverter grid voltage is high ·
6 Inverter grid under frequency · 7 Inverter grid over frequency ·
8 Inverter over current protection error · 9 Inverter bus voltage is too low ·
10 Inverter soft start failed · 11 Over DC voltage in AC output ·
12 Battery connection is open · 13 Inverter control current sensor error ·
14 Inverter output voltage is too low · 15 (unused)

### 7.3 Inverter Warning1 (25265), bits 0–15

0 Fan is locked when inverter is on · 1 Fan2 is locked when inverter is on ·
2 Battery is over-charged · 3 Low battery · 4 Overload ·
5 Output power derating · 6 Solar charger stops due to low battery ·
7 Solar charger stops due to high PV voltage ·
8 Solar charger stops due to over load · 9 Solar charger over temperature ·
10 PV charger communication error · 11–15 (unused)

### 7.4 Charger Error1 (15213), bits 0–15

0 Hardware protection · 1 Over current · 2 Current sensor error ·
3 Over temperature · 4 PV voltage is too high · 5 (unused) ·
6 Battery voltage is too high · 7 Battery voltage is too low ·
8 Current is uncontrollable · 9 Parameter error · 10–15 (unused)

### 7.5 Charger Warning1 (15214), bits 0–15

0 Fan error · 1–15 (unused)

---

## 8. Writing Settings

No write frames appear in the capture (settings were not changed), but the
monitor's model makes the write surface explicit:

* **Writable registers** = `Ph1800M.AffectAddress` (22 registers):
  10103, 10104, 10105, 10108, 10110 (charger) and
  20101, 20102, 20103, 20104, 20108, 20109, 20111, 20112, 20113,
  20118, 20119, 20125, 20127, 20128, 20132, 20142, 20143 (inverter).
* Standard Modbus RTU function codes apply: **FC 0x06** (write single
  register) and **FC 0x10** (write multiple registers). Values are sent with
  the same scaling as on read (e.g. 230.0 V → `0x08FC` to 20102).
* Command-style registers (write a value to trigger an action):
  * `10112` RemoveTheAccumulatedData — clears accumulated energy counters;
  * `10124` EqualizationActivedImmediately — start equalization now;
  * `10126` ResetTheParameter — factory-reset charger parameters.
* After a successful write the monitor re-reads the affected block
  (`AffectAddress` entries belong to the 10103 and 20101 blocks).

Example (FC 0x06, set inverter output voltage to 230.0 V):

```
04 06 4E 86 08 FC <CRC-L> <CRC-H>
```

Expected echo: `04 06 4E 86 08 FC <CRC-L> <CRC-H>` (standard Modbus RTU echo).

---

## 9. Sample Decoded Session (27/08/2026 capture)

State of the captured unit at 13:43:01:

| Group | Value |
|---|---|
| Model | PV1800 (edition 1.04.14, HW 10.101, SW 49.05) |
| Inverter work state | **OffGrid** (2) |
| Battery | 12 V class, 12.6 V, 1 A, 20 W; Li battery 200 Ah |
| Inverter output | 230.0 V / 50.00 Hz, 0 W, 100 VA |
| DC bus | 463.0 V |
| Temperatures | AC heatsink 60 °C, DC heatsink 76 °C, charger heatsink 44 °C |
| Relays | inverter ON, load ON, grid OFF, N-line OFF, DC ON, earth OFF |
| Charger | Selftest mode, MPPT stop, PV 0 V, charger current 0 A |
| Settings | off-grid enabled, 230 V/50 Hz, UTI mode, Home protection, max discharge 6.6 A, stop discharge 11.5 V, stop charge 14.2 V, grid charge max 20 A, low 11.0 V, high 15.0 V, combined charge max 60 A, charger priority "Solar and Utility" |
| Energy | discharge 1.4 kWh, load 15.6 kWh, self-use 1.4 kWh, PV 0.2 kWh |
| Errors/warnings | none active (one undefined charger-error bit 5 set) |

---


### Open questions / caveats

1. Registers marked "(unmapped)" exist in the frame but have no name in the
   monitor; several look like mirrored copies of neighboring settings
   (e.g. 20114 = 20113, 20126 = 20125, 20131 = 20118).
2. The `2xxxx`/`1xxxx` "C" registers (20009–20016, 10006–10008) carry raw
   counts; the monitor never displays them — they may be firmware-internal.
3. Write behavior (FC 0x06/0x10) is inferred from the Modbus RTU standard and
   the monitor's `AffectAddress` list, not observed on the wire.
4. Only slave ID 4 was active in the capture; parallel operation
   (25229–25231) was not exercised.
