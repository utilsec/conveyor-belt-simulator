# Conveyor Line PLC Simulator

`conveyor_belt.py` · Python 3.8+ · standard library only · Windows / Linux / macOS

A self-contained simulated PLC for one zone of a manufacturing line: a
motor-driven conveyor belt with a VFD, infeed/outfeed photo-eyes, a pneumatic
reject kicker, E-stop and guard-door interlocks, a stack light, and a live
process model (speed ramps, motor current and winding temperature, part
counting, throughput, vibration, and a latching thermal-overload trip).

It exists to be a **safe target**. Point a Modbus client, a scanner, or
[AtomICS](https://github.com/utilsec/atomics) at it, read and write its coils
and registers, and watch a believable machine respond. No plant, no risk. It is
just as useful as a teaching prop: the browser HMI shows the process reacting in
real time as you change setpoints.

Two interfaces share **one** live data model, so they never disagree:

* **Modbus/TCP** on port 502
* **Web HMI + JSON API** on port 60000

One file, nothing to install: `python conveyor_belt.py` runs from a USB stick on
a lab box with no internet.

> *(formerly `plc_sim.py`. The module docstring still carries the old name.)*

## ⚠️ Disclaimer

For training/education or **AUTHORIZED** testing only, on systems you own or are
explicitly permitted to use. This is a deliberately insecure simulated
controller: its Modbus server and HTTP API accept unauthenticated reads and
writes from anyone who can reach them. Run it on an isolated lab network or bound
to localhost. It **must not** be run on a production network or anywhere safety
and/or operational availability cannot be guaranteed. You are solely responsible
for how it is used.

## Quick start

```
python conveyor_belt.py                                   # 0.0.0.0 - Modbus 502 + HMI 60000
python conveyor_belt.py --modbus-port 5020                # unprivileged Modbus port
python conveyor_belt.py --host 127.0.0.1 --hmi-port 8080  # localhost only, HMI on 8080
python conveyor_belt.py --speed 90 -v                     # start with an overload setpoint, log traffic
python conveyor_belt.py --help                            # all options
```

Then open **http://localhost:60000/** for the HMI, or point a Modbus client at
`localhost:502`. `Ctrl+C` stops it.

* **Linux:** ports below 1024 (so the default 502) need root or a granted
  capability. Use `--modbus-port 5020` to run as a normal user. If the Modbus
  bind is refused, the sim tells you so and exits.
* **Windows:** no admin needed for 502. Allow inbound TCP 502 and 60000 through
  Windows Firewall if you're reaching the sim from another host.

The startup banner prints the bind addresses, the key coil/register handles, and
the overload warning/trip times at 80–100 % so you know what to expect before
you touch anything.

## The simulated process

**Conveyor Line — Zone 1** (`PLC-01`) is a single belt driven by a VFD. With the
interlocks satisfied, a run command ramps the belt to the speed setpoint over the
accel time (HR2) and back down over the decel time (HR3). A setpoint of 1–4 %
runs at the drive's 5 % minimum speed.

Parts are fed at the target throughput (HR1), limited by belt capacity
(180 parts/min at 100 %), and only while both upstream and downstream report
ready. Each part trips the infeed eye, travels the belt (faster belt, shorter
trip), and is counted at the outfeed. A share set by the defect rate (HR5) is
defective. Those are pulsed off by the reject kicker and counted as rejects, but
only if air pressure is OK and the reject bin isn't full.

Motor current, VFD frequency (60 Hz at 100 %), winding temperature, line-voltage
droop, vibration, throughput (60 s rolling window), runtime and scan time all
move with the process on a 10 Hz tick (`--rate`), so polled values change the
way real ones would.

The REVERSE coil flips the HMI's direction indicator and mimic. The process
model behaves the same in either direction.

### Safety interlocks (PLC logic)

The belt only runs when it is **safe**: E-stop clear, guard door closed, and no
latched overload. If any of those goes bad, the PLC drops the RUN coil and the
belt ramps down. RUN **stays off**: clearing the condition does not restart the
line, and the run command has to be issued again, as on a real machine.

The stack light is PLC-driven:

| Lamp | Lit when |
|---|---|
| Red | overload trip or E-stop active |
| Amber | overtemp warning, reject bin full, low air pressure, or downstream not ready (and no fault) |
| Green | belt running with no fault |

An open guard door stops the line and raises a critical alarm on the HMI, but it
does not light the red lamp.

You can exercise all of this from the HMI's E-STOP button and Field I/O card
without a Modbus client.

### The overload / thermal model (the interesting part)

The motor's **continuous duty rating is 75 % belt speed**. At or below it the
line runs indefinitely. At 75 % the winding settles around 60 °C after a few
minutes, well short of any alarm. Above it the motor banks "thermal load" at a
rate set by how far past the rating it is running:

```
load rate = (speed − 75) / (100 − 75)   per second      warning at 5, trip at 10
```

| Belt speed | Overtemp warning | Critical trip |
|---|---|---|
| ≤ 75 % | never | never |
| 76 % | ~125 s | ~250 s |
| 80 % | ~25 s | ~50 s |
| 85 % | ~12.5 s | ~25 s |
| 90 % | ~8 s | ~17 s |
| 95 % | ~6 s | ~12.5 s |
| 100 % | ~5 s | ~10 s |

Times count from the moment the belt passes 75 %, not from the setpoint write;
the VFD ramp adds a second or two. Drop back to 75 % or below before the trip and
the accumulated load bleeds off at 0.5 per second.

**At the trip** the winding reads the critical temperature (HR7, 140 °C by
default; Class F insulation is rated to a 155 °C hotspot), MOTOR_OVERLOAD (DI 3)
latches, the PLC drops RUN and forces the conveyor to stop, the red lamp lights,
and the HMI shows the trip reason.

**Fault Reset** (coil 6) is refused until the winding has cooled 45 °C below the
trip temperature (95 °C at defaults). That's about 45 s with the cooling fan off,
or about 25 s with it on. The fan also shaves a little off the winding
temperature during an overload, but it doesn't change the trip timing.

The two temperature setpoints are live too. Reaching HR6 raises the warning, and
reaching HR7 by any route trips the line. Because the reset threshold moves with
HR7, a lowered trip setpoint also keeps the line locked out.

It's a clean demo of a process that looks fine for a while and then bites, and
of why a "just push it faster" setpoint write is dangerous.

## Modbus/TCP

0-based PDU addressing. Supported function codes: **1, 2** (read coils /
discrete inputs), **3, 4** (read holding / input registers), **5, 6** (write
single coil / register), **15, 16** (write multiple).

| Request | Response |
|---|---|
| Any other function code, including FC 43/14 Read Device Identification and FC 17 Report Server ID | exception **01** Illegal Function |
| Range past the end of a bank, or a quantity over 2000 bits / 125 registers | exception **02** Illegal Data Address |
| FC 5 with a value other than `0xFF00` / `0x0000`, or a truncated PDU | exception **03** Illegal Data Value |

* **Unit ID:** the banner and HMI say unit 1, but the sim answers on **any** unit
  ID and echoes it back.
* **Banks:** each bank (coils, discrete inputs, holding, input registers) is 256
  entries. Addresses outside the map below are accepted (unmapped addresses read
  0 until written) but don't drive anything.
* **Connections:** multiple clients can connect at once. An idle connection is
  closed after 120 s.

**Coils (FC 01/05/15)**

| Addr | Tag | Notes |
|---|---|---|
| 0 | `RUN` | operator: conveyor run command; dropped by the PLC on any interlock trip |
| 1 | `REVERSE` | operator: belt direction (indicator only) |
| 2, 4, 5 | — | reserved |
| 3 | `COOLING_FAN` | operator: panel cooling fan |
| 6 | `FAULT_RESET` | operator: momentary (self-clears); refused while the winding is hot |
| 7 | `KICKER_TEST` | operator: momentary pulse of the reject kicker |
| 8 / 9 / 10 | `BEACON_GREEN / AMBER / RED` | PLC-driven stack light. Outside writes are acknowledged but ignored. |

**Discrete inputs (FC 02)**

| Addr | Tag | Startup |
|---|---|---|
| 0 | `ESTOP` (1 = E-stop active) | 0 |
| 1 | `INFEED_EYE` | 0 |
| 2 | `OUTFEED_EYE` | 0 |
| 3 | `MOTOR_OVERLOAD` (latched) | 0 |
| 4 | — reserved | 0 |
| 5 | `GUARD_CLOSED` (1 = safe) | 1 |
| 6 | `UPSTREAM_READY` | 1 |
| 7 | `DOWNSTREAM_READY` | 1 |
| 8 | `REJECT_BIN_FULL` | 0 |
| 9 | `AIR_PRESSURE_OK` | 1 |

**Holding registers (FC 03/06/16): setpoints**

| Addr | Tag | Units | Default |
|---|---|---|---|
| 0 | `SPEED_SETPOINT` | % (0–100) | 75 |
| 1 | `TARGET_THROUGHPUT` | parts/min | 120 |
| 2 | `ACCEL_TIME` | ms, 0→100 % (100 ms floor) | 2500 |
| 3 | `DECEL_TIME` | ms, 100→0 % (100 ms floor) | 1500 |
| 4 | `BATCH_SIZE` | parts | 500 |
| 5 | `DEFECT_RATE` | parts per thousand | 20 |
| 6 | `TEMP_WARN` | °C | 100 |
| 7 | `TEMP_CRITICAL_TRIP` | °C (latched, stops conveyor) | 140 |

**Input registers (FC 04): live values, read-only**

| Addr | Tag | Scaling |
|---|---|---|
| 0 | `BELT_SPEED` | % ×10 |
| 1 | `MOTOR_CURRENT` | A ×10 |
| 2 | `MOTOR_TEMP` | °C ×10 |
| 3 | `VFD_FREQ` | Hz ×10 |
| 4 | `LINE_VOLTAGE` | V (480 nominal) |
| 5 / 6 | `PARTS_TOTAL` | low / high word |
| 7 | `REJECTS_TOTAL` | count |
| 8 | `THROUGHPUT` | parts/min |
| 9 | `RUNTIME` | hours ×10 |
| 10 | `VIBRATION` | mm/s ×100 |
| 11 | `AMBIENT` | °C ×10 |
| 12 | `SCAN_TIME` | ms ×10 |
| 13 | `BATCH_COUNT` | parts in current batch |

> **Holding registers aren't range-checked on a Modbus write. That's
> deliberate.** Write 500 to HR0 and it reads back 500, which is a realistic
> unvalidated setpoint write. The process model only acts on 0–100 %, so the belt
> still tops out at 100 %. The HMI and `/api/speed` clamp the speed; a
> hand-crafted Modbus write doesn't have to.

## Web HMI (port 60000)

A single page that polls the sim every 400 ms:

* **Process mimic**: the animated belt, PE-01 / PE-02 photo-eyes, reject kicker
  and chute, motor/VFD, direction, and commanded vs actual speed. Live readouts
  below it cover speed, setpoint, VFD frequency, current, winding temp,
  throughput, parts, rejects, vibration, line voltage, runtime and batch.
* **Alarm banner**: critical alarms (E-stop, overload trip with its reason and
  reset temperature, guard door open) outrank warnings (overtemp with a
  seconds-to-trip countdown, reject bin full, low air).
* **Header**: stack light, link status, clock, and PLC scan time.
* **Conveyor Control**: Start/Stop Conveyor Belt, E-STOP (latching), direction
  reverse, cooling fan, Fault Reset, Test Kicker.
* **Speed Command · VFD**: slider, numeric entry, −5 / −1 / +1 / +5 nudges, and
  presets (Stop, Jog 15, Slow 40, Rated 75, Max 100). Up/down arrow keys nudge
  the speed; hold Shift for 10 %. Above the 75 % rating a thermal-load bar shows
  progress toward the warning and trip marks with a countdown, and it stays up
  while the load bleeds off. Accel/decel ramp fields sit below.
* **Setpoints**: target throughput, batch size, defect rate, warning temp,
  critical trip temp.
* **Field I/O · Simulate Sensors**: guard door, air pressure, upstream and
  downstream ready, reject bin full.
* **Theme**: dark/light panel toggle. Your choice is remembered in the browser;
  a first visit follows the OS light/dark setting.

## JSON API (port 60000)

Same data, scriptable without a Modbus library. Like the Modbus side, it has no
authentication.

| Method / path | Purpose |
|---|---|
| `GET /api/state` | Full live snapshot: process values, coils, inputs, faults (including thermal load and trip reason) |
| `GET /api/map` | The coil / register map, as JSON |
| `POST /api/speed` | Belt-speed command: `{"value": 80}` absolute or `{"delta": 5}` relative; clamped to 0–100 % |
| `POST /api/coil` | Set a coil: `{"addr": 0, "value": true}`. Beacon coils are refused (`"ok": false`). |
| `POST /api/register` | Write a holding register: `{"addr": 2, "value": 1500}`. Not range-checked. |
| `POST /api/input` | Flip a discrete input (E-stop, guard, …): `{"addr": 0, "value": true}` |

Writing DI 3 through `/api/input` sets or clears the overload latch directly,
skipping the cool-down check. It's handy for injecting a fault in a demo.

## Options

```
-h, --help          show the options and exit
--host ADDR         bind address for both servers (default 0.0.0.0)
--modbus-port N     Modbus/TCP port (default 502; 5020 runs unprivileged on Linux)
--hmi-port N        web HMI + JSON API port (default 60000)
--rate HZ           simulation ticks per second (default 10)
--speed N           initial belt speed setpoint, 0-100 % (default 75)
-v, --verbose       log Modbus connects/disconnects and HMI requests
```

## Test procedure

Examples use [mbpoll](https://github.com/epsilonrt/mbpoll) (1-based references,
so `-r 1` is address 0) and the sim on `127.0.0.1:5020`. Any Modbus client
works, including AtomICS (**Protocol Modules → Modbus**; writes need
`--active`).

1. **Start it.** `python conveyor_belt.py --modbus-port 5020`. The banner shows
   the bind addresses and the warning/trip table.
2. **Run at rated speed.** Open http://localhost:60000/ and click **Start
   Conveyor Belt**. The belt ramps to 75 %, the green lamp lights, parts count,
   and throughput climbs toward 120 parts/min. Leave it running; it never warns.
3. **Read it.** `mbpoll -m tcp -p 5020 -a 1 -t 3 -r 1 -c 14 -1 127.0.0.1` reads
   IR 0–13 (IR 0 reads `750` = 75.0 %).
   `mbpoll -m tcp -p 5020 -a 1 -t 0 -r 1 -c 11 -1 127.0.0.1` reads coils 0–10
   (RUN and BEACON_GREEN set).
4. **The write that bites.** `mbpoll -m tcp -p 5020 -a 1 -t 4 -r 1 127.0.0.1 100`
   writes 100 to HR0. About 5 s after the belt passes 75 %, the lamp goes amber
   and the HMI shows OVERTEMP WARNING with a countdown. About 10 s after, the lamp
   goes red, DI 3 reads 1, RUN drops, and the belt stops.
5. **Reset.** Click **Fault Reset** straight away; it's refused. Wait until the
   winding is below 95 °C (about 45 s, or about 25 s with the cooling fan on).
   Reset clears the fault, and the belt stays stopped until you start it again.
6. **Interlocks.** While running, open the guard door (Field I/O). The belt stops
   and RUN drops. Close the guard and the belt stays stopped until you click
   Start. Repeat with **E-STOP** (red lamp).
7. **Setpoint tampering.** While running at 75 %, write 50 to HR7 with
   `mbpoll -m tcp -p 5020 -a 1 -t 4 -r 8 127.0.0.1 50`. The line trips
   immediately, and Fault Reset stays refused until HR7 is put back to 140.
8. **Protocol edges.** FC 43 Read Device Identification returns exception 01
   (AtomICS reports *device did not respond to FC 43*). A read past address 255
   returns exception 02. A request to unit ID 7 is answered as unit 7.

Pair it with [MOTH](https://github.com/utilsec/moth) to compare a responding
PLC against a honeypot that alerts on the same traffic.

## Operational notes

* **Insecure by design.** It has no authentication on Modbus or HTTP, and both
  servers bind all interfaces by default. Use `--host 127.0.0.1` for
  single-machine demos, or keep it on an isolated lab segment.
* It never initiates traffic. It only answers.
* It's a lab target, not a sensor. It doesn't log or alert on who talked to it
  (that's MOTH's job); `-v` prints connects and HTTP requests to the console.
* If a port is already in use (say, another sim on 502 or 60000), startup stops
  with Python's *address already in use* error. Move it with `--modbus-port` /
  `--hmi-port`.

---

Vibe coded by Mike Holcomb of UtilSec, LLC ·
[LinkedIn](https://www.linkedin.com/in/mikeholcomb) · [mikeholcomb.com](https://mikeholcomb.com)

AUTHORIZED USE ONLY. For training/education and authorized testing on isolated
lab networks; not for production environments where safety or operational
availability cannot be guaranteed.
