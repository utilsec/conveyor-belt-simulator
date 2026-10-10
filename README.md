# Conveyor Line PLC Simulator

`conveyor_belt.py` · Python 3.8+ · standard library only · Windows / Linux / macOS

A self-contained simulated PLC for one zone of a manufacturing line: a
motor-driven conveyor belt with a VFD, infeed/outfeed photo-eyes, a pneumatic
reject kicker, E-stop and guard-door interlocks, a stack light, and a live
process model (speed ramps, motor current and winding temperature, part
counting, throughput, vibration, and a latching thermal-overload trip).

It exists to be a **safe target**. Point a Modbus client, a scanner, or AtomICS
at it, read and write its coils and registers, and watch a believable machine
respond — no plant, no risk. It's equally a teaching prop: the browser HMI shows
the process reacting in real time as you change setpoints.

Two interfaces share **one** live data model, so they never disagree:

* **Modbus/TCP** on port 502 — the standard PLC fieldbus
* **Web HMI + JSON API** on port 60000

Pure standard library — no packages to install.

## Quick start

```
python conveyor_belt.py                                   # 0.0.0.0 — Modbus 502 + HMI 60000
python conveyor_belt.py --modbus-port 5020                # unprivileged (no root/admin needed)
python conveyor_belt.py --host 127.0.0.1 --hmi-port 8080  # bind local, move the HMI
python conveyor_belt.py --speed 90 -v                     # start overloaded, log connections
```

Then open **http://localhost:60000/** for the HMI, or talk Modbus to
`localhost:502`. Ports below 1024 (so the default 502) need root/admin or a
granted capability — use `--modbus-port 5020` to run as a normal user. `Ctrl+C`
stops it.

The startup banner prints the bind addresses, the key coil/register handles, and
the overload warn/trip times at 80–100 % so you know what to expect before you
touch anything.

## The simulated process

`Conveyor Line — Zone 1`: a single belt driven by a VFD. Commanding a run with
the interlocks satisfied ramps the belt up over the accel time; parts arrive at
the infeed eye, travel the belt, and are counted at the outfeed; a configurable
share are "defective" and pulsed off by the reject kicker (if air is OK and the
reject bin isn't full). Motor current, VFD frequency, winding temperature,
vibration, throughput and runtime all move with belt speed on a 10 Hz tick, so
polled values change the way real ones would.

### Safety interlocks (PLC logic)

The belt only runs when it is **safe** — E-stop clear, guard door closed, and no
latched overload. Trip any of those and the PLC forces the conveyor to stop; the
stack light follows (red on fault or E-stop, amber on a warning condition, green
while running clean). You can flip the field sensors from the HMI to exercise
this — press the virtual E-stop, open the guard — without a Modbus client.

### The overload / thermal model (the interesting part)

The motor's **continuous duty rating is 75 % belt speed**. At or below it the
line runs indefinitely and the winding settles around 60–70 °C. Above it the
motor banks "thermal load" at a rate set by how far past the rating it's running:

```
load rate = (speed − 75) / (100 − 75)   per second
```

So 100 % warns at ~5 s and trips at ~10 s, 90 % at ~8 s / ~17 s, 80 % at ~25 s /
~50 s — every speed over 75 % trips eventually, nothing at or below it ever does.
The trip latches at a critical winding temperature (140 °C, Class F insulation),
forces the conveyor to stop, and **won't reset** until the motor has cooled well
below the trip point (about 45 °C of margin). It's a clean demo of a process that
looks fine for a while and then bites — good for showing why a "just push it
faster" setpoint write is dangerous.

## Modbus map (0-based PDU addressing, unit id 1)

Supported function codes: **1, 2** (read coils / discrete inputs), **3, 4** (read
holding / input registers), **5, 6** (write single coil / register), **15, 16**
(write multiple). Out-of-range or malformed requests get the correct Modbus
exception (illegal address / value / function) rather than a crash.

**Coils (FC 01/05/15) — discrete outputs**

| Addr | Tag | |
|---|---|---|
| 0 | `RUN` | operator: conveyor run command |
| 1 | `REVERSE` | operator: belt direction |
| 3 | `COOLING_FAN` | operator: panel cooling fan |
| 6 | `FAULT_RESET` | operator: momentary — clears latched faults |
| 7 | `KICKER_TEST` | operator: manual pulse of the reject kicker |
| 8 / 9 / 10 | `BEACON_GREEN / AMBER / RED` | PLC-driven stack light (read-only to outside writers) |

**Discrete inputs (FC 02) — field sensors / status**

| Addr | Tag | | Addr | Tag |
|---|---|---|---|---|
| 0 | `ESTOP` active | | 6 | `UPSTREAM_READY` |
| 1 | `INFEED_EYE` | | 7 | `DOWNSTREAM_READY` |
| 2 | `OUTFEED_EYE` | | 8 | `REJECT_BIN_FULL` |
| 3 | `MOTOR_OVERLOAD` (latched) | | 9 | `AIR_PRESSURE_OK` |
| 5 | `GUARD_CLOSED` (safe) | | | |

**Holding registers (FC 03/06/16) — setpoints, read/write**

| Addr | Tag | | Addr | Tag |
|---|---|---|---|---|
| 0 | `SPEED_SETPOINT` % (0–100) | | 4 | `BATCH_SIZE` |
| 1 | `TARGET_THROUGHPUT` parts/min | | 5 | `DEFECT_RATE` permille |
| 2 | `ACCEL_TIME` ms | | 6 | `TEMP_WARN` °C |
| 3 | `DECEL_TIME` ms | | 7 | `TEMP_CRITICAL_TRIP` °C (latched, stops conveyor) |

**Input registers (FC 04) — live process values, read-only.** Scaled integers:
belt speed, motor current, winding temp, VFD freq are ×10; vibration is ×100;
line voltage and counters are raw. `PARTS_TOTAL` is a hi/lo pair (IR 5/6).

| Addr | Tag | | Addr | Tag |
|---|---|---|---|---|
| 0 | `BELT_SPEED` %×10 | | 7 | `REJECTS_TOTAL` |
| 1 | `MOTOR_CURRENT` A×10 | | 8 | `THROUGHPUT` parts/min |
| 2 | `MOTOR_TEMP` °C×10 | | 9 | `RUNTIME` hrs×10 |
| 3 | `VFD_FREQ` Hz×10 | | 10 | `VIBRATION` mm/s×100 |
| 4 | `LINE_VOLTAGE` V | | 11 | `AMBIENT` °C×10 |
| 5 / 6 | `PARTS_TOTAL` low / high | | 12 | `SCAN_TIME` ms×10 |
| | | | 13 | `BATCH_COUNT` |

> **A raw Modbus write to HR0 is deliberately not clamped** — an unvalidated
> setpoint write is realistic, and the process model limits what it actually acts
> on. The HMI's speed station clamps to 0–100 %; a hand-crafted Modbus write
> doesn't have to.

## Web HMI + JSON API (port 60000)

The HMI renders the live process and lets you drive it: a **Speed Command**
station (slider, numeric entry, ±1 / ±5 nudge, presets, and up/down arrow keys —
shift = 10 %), accel/decel ramp fields, run/stop, fault reset, and toggles for
the field sensors. It has a dark/light theme toggle (dark is the default).

The same data is available as JSON, so you can script against it without a Modbus
library:

| Method / path | Purpose |
|---|---|
| `GET /api/state` | Full live snapshot of the model |
| `GET /api/map` | The coil / register map above, as JSON |
| `POST /api/speed` | Belt-speed command — `{"value": 80}` absolute or `{"delta": 5}` relative |
| `POST /api/coil` | Set an operator coil — `{"addr": 0, "value": true}` |
| `POST /api/register` | Write a holding register — `{"addr": 2, "value": 1500}` |
| `POST /api/input` | Flip a field sensor (E-stop, guard, …) — `{"addr": 0, "value": true}` |

## Options

```
--host ADDR            bind address (default 0.0.0.0)
--modbus-port N        Modbus/TCP port (default 502; use 5020 to run unprivileged)
--hmi-port N           web HMI + API port (default 60000)
--rate HZ              simulation ticks per second (default 10)
--speed N              initial belt speed setpoint, 0-100 % (default 75)
-v, --verbose          log Modbus connects/disconnects and HMI requests
```

## Using it as a test target

It's the safe end of the portfolio's tooling — anything that speaks Modbus or
scans a host can be aimed at it:

* **AtomICS** → Protocol Modules → Modbus: read coils/registers, scan, Read
  Device Identity, recon overview; in `--active` mode, exercise a write against
  it and watch the process react (drop the speed setpoint, force a run, trip the
  overload). Network Discovery will also find it on 502.
* **MOTH** — run both to compare a real-ish responding PLC against the honeypot.
* Any Modbus client (`mbpoll`, pymodbus, a SCADA test harness) — standard FC
  1/2/3/4/5/6/15/16, so it just works.

A good first exercise: open the HMI, start the belt at 75 %, confirm it runs
forever, then write `100` to HR0 over Modbus and watch it warn at ~5 s and trip
at ~10 s — a write that looks harmless and isn't.

---

> **Note:** the file on disk is `conveyor_belt.py`; its internal docstring and
> banner still refer to the tool as `plc_sim.py` from before the rename. Harmless,
> but if you want them to match say the word and I'll update the header, usage
> lines, and banner.
