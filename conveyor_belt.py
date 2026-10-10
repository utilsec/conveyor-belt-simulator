#!/usr/bin/env python3
"""
plc_sim.py  --  Simulated manufacturing-line PLC

Simulates a programmable logic controller for one zone of a manufacturing
plant: a motor-driven conveyor belt with infeed/outfeed photo-eyes, a VFD,
a pneumatic reject kicker, safety interlocks (E-stop, guard door), a stack
light, and a running process model (speed ramps, motor current/temperature,
part counting, throughput, vibration, faults).

Two interfaces share ONE live data model, so they always agree:

  * Modbus/TCP server on port 502   (standard PLC fieldbus)
  * Web HMI + JSON API on port 60000

Pure Python standard library -- no external packages required.

    python3 plc_sim.py                 # 0.0.0.0:502 + 0.0.0.0:60000
    python3 plc_sim.py --host 127.0.0.1 --modbus-port 5020 --hmi-port 60000

Ports below 1024 (e.g. 502) need root/admin or a granted capability.
Use --modbus-port 5020 to run unprivileged.

Belt speed is commanded from the HMI "Speed Command" panel (slider, numeric
entry, +/- nudge, and preset buttons) or by writing holding register 0 over
Modbus.  The VFD ramp times (HR2 accel / HR3 decel) are adjustable too, so
you can watch the belt accelerate and coast down instead of stepping.

The motor's continuous duty rating is 75 % belt speed: at or below it the line
runs indefinitely and winding temperature settles around 60-70 C.  Above it the
motor is in overload and banks heat at a rate set by how far past the rating it
is running, so every speed over 75 % trips eventually -- 100 % warns at 5 s and
trips at 10 s, 90 % at about 8 s and 17 s, 80 % at about 25 s and 50 s.  The
trip is latched at a critical winding temperature (140 C, Class F), the PLC
forces the conveyor to stop, and the fault will not reset until the motor has
cooled well below the trip point.
"""

import argparse
import json
import math
import random
import socket
import socketserver
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --------------------------------------------------------------------------
# Tag / register map  (0-based Modbus PDU addressing)
# --------------------------------------------------------------------------
# Coils (FC 01/05/15) -- discrete outputs
COIL_RUN          = 0   # operator: conveyor run command
COIL_REVERSE      = 1   # operator: belt direction (1 = reverse)
# coil 2 reserved (formerly DIVERTER gate)
COIL_COOLING_FAN  = 3   # operator: panel cooling fan
# coil 4 reserved (formerly HYD_PUMP)
# coil 5 reserved (formerly LABELER)
COIL_FAULT_RESET  = 6   # operator: momentary -- clears latched faults
COIL_KICKER_TEST  = 7   # operator: manual pulse of reject kicker
COIL_BEACON_GREEN = 8   # PLC-driven indicator (stack light: running)
COIL_BEACON_AMBER = 9   # PLC-driven indicator (stack light: warning)
COIL_BEACON_RED   = 10  # PLC-driven indicator (stack light: fault)

OPERATOR_COILS = {COIL_RUN, COIL_REVERSE, COIL_COOLING_FAN,
                  COIL_FAULT_RESET, COIL_KICKER_TEST}

# Discrete inputs (FC 02) -- field sensors / status
DI_ESTOP          = 0   # 1 = E-stop ACTIVE (mushroom pressed)
DI_INFEED_EYE     = 1   # 1 = part at infeed photo-eye
DI_OUTFEED_EYE    = 2   # 1 = part at outfeed photo-eye
DI_MOTOR_OVERLOAD = 3   # 1 = thermal overload tripped (latched)
# discrete input 4 reserved (formerly BELT_JAM detect)
DI_GUARD_CLOSED   = 5   # 1 = guard door closed (safe)
DI_UPSTREAM_RDY   = 6   # 1 = upstream line ready to feed
DI_DOWNSTREAM_RDY = 7   # 1 = downstream line ready to accept
DI_REJECT_FULL    = 8   # 1 = reject bin full
DI_AIR_OK         = 9   # 1 = pneumatic air pressure OK

# Holding registers (FC 03/06/16) -- setpoints, read/write
HR_SPEED_SP       = 0   # belt speed setpoint, 0-100 %
HR_TARGET_TPM     = 1   # target throughput, parts/min
HR_ACCEL_MS       = 2   # speed-up ramp time, ms (0..100%)
HR_DECEL_MS       = 3   # slow-down ramp time, ms
HR_BATCH_SIZE     = 4   # parts per batch
HR_DEFECT_PERMILLE= 5   # simulated defect rate, parts-per-thousand
HR_TEMP_WARN      = 6   # motor temp warning, deg C
HR_TEMP_TRIP      = 7   # motor temp overload trip, deg C

# Input registers (FC 04) -- live process values, read-only
IR_BELT_SPEED     = 0   # actual belt speed, % x10
IR_MOTOR_CURRENT  = 1   # motor current, A x10
IR_MOTOR_TEMP     = 2   # motor winding temp, deg C x10
IR_VFD_FREQ       = 3   # VFD output frequency, Hz x10
IR_LINE_VOLTAGE   = 4   # measured line voltage, V
IR_PARTS_LOW      = 5   # total parts produced, low word
IR_PARTS_HIGH     = 6   # total parts produced, high word
IR_REJECTS        = 7   # total parts rejected
IR_THROUGHPUT     = 8   # actual throughput, parts/min
IR_RUNTIME_HRS    = 9   # motor runtime, hours x10
IR_VIBRATION      = 10  # bearing vibration, mm/s x100
IR_AMBIENT_TEMP   = 11  # ambient temp, deg C x10
IR_SCAN_MS        = 12  # PLC scan time, ms x10
IR_BATCH_COUNT    = 13  # parts in current batch

BANK_SIZE = 256
RATED_CURRENT_A = 12.0     # motor rated current
SCALE_SPEED  = 10
SCALE_TEMP   = 10
SCALE_FREQ   = 10
SCALE_VIB    = 100
SCALE_HRS    = 10
SCALE_SCAN   = 10

# Belt speed command limits (percent).  MIN_RUN_SPEED_PCT is the lowest speed
# the VFD will hold while running -- below it the drive is effectively stopped.
SPEED_MIN_PCT = 0
SPEED_MAX_PCT = 100
MIN_RUN_SPEED_PCT = 5

# --- Overload thermal model --------------------------------------------
# CONTINUOUS_SPEED_PCT is the motor's continuous duty rating: at or below it
# the machine runs indefinitely and winding temperature settles well short of
# any alarm.  Above it the motor is in overload and accumulates "thermal load"
# -- measured in equivalent seconds at 100 % -- at a rate proportional to how
# far past the rating it is running:
#
#     load rate = (speed - 75) / (100 - 75)
#
# So 100 % accrues 1.0/s (warning at 5 s, trip at 10 s), 90 % accrues 0.6/s
# (warning ~8 s, trip ~17 s), and 80 % accrues 0.2/s (trip ~50 s).  Every
# speed above the rating trips eventually; nothing at or below it ever does.
# Class F insulation is rated to a 155 C hotspot, so 140 C is a credible
# critical winding temperature and 100 C a credible warning level.
CONTINUOUS_SPEED_PCT = 75.0      # continuous duty rating -- safe forever
OVERLOAD_EXP = 1.0               # >1 = overload bites harder near 100 %
THERMAL_LOAD_WARN = 5.0          # staged warning at this much load
THERMAL_LOAD_TRIP = 10.0         # critical trip / forced stop
THERMAL_LOAD_RECOVERY = 0.5      # load shed per second at/below the rating
OVERSPEED_CURVE = 1.35           # >1 = heating accelerates as it climbs
TEMP_WARN_DEFAULT_C = 100        # HR6 default
TEMP_CRIT_DEFAULT_C = 140        # HR7 default -- critical winding temp
TEMP_RESET_MARGIN_C = 45         # must cool this far below trip to reset


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# --------------------------------------------------------------------------
# Shared, thread-safe data model + process simulation
# --------------------------------------------------------------------------
class PlantModel:
    """Holds the four Modbus data banks plus the analog process state.
    All access is guarded by a single re-entrant lock."""

    def __init__(self):
        self.lock = threading.RLock()
        self.coils = [False] * BANK_SIZE
        self.discrete = [False] * BANK_SIZE
        self.holding = [0] * BANK_SIZE
        self.input_regs = [0] * BANK_SIZE

        # --- analog / internal process state (floats) ---
        self.speed = 0.0           # actual belt speed, %
        self.ambient = 24.0        # deg C
        self.motor_temp = self.ambient
        self.current = 0.0         # A
        self.vibration = 0.0       # mm/s
        self.line_voltage = 480.0
        self.parts = 0
        self.rejects = 0
        self.batch_count = 0
        self.runtime_s = 0.0
        self.scan_ms = 10.0
        self.throughput = 0.0

        # latched faults
        self.fault_overload = False
        self.warn_overtemp = False      # staged (5 s) overtemp warning
        self.trip_reason = ""           # what caused the last critical trip

        # overload thermal accumulator
        self._thermal_load = 0.0        # equivalent seconds at 100 % speed
        self._load_rate = 0.0           # current accumulation rate, per second
        self._overload_t0_temp = None   # winding temp when the overload began

        # pulse / timing helpers
        self._part_accum = 0.0
        self._in_flight = []       # list of [arrive_time, defective]
        self._infeed_timer = 0.0
        self._outfeed_timer = 0.0
        self._kicker_timer = 0.0
        self._prod_window = []     # timestamps of recent completions (throughput)
        self._sim_time = 0.0

        self._init_defaults()

    def _init_defaults(self):
        # field inputs that are "healthy" at startup
        self.discrete[DI_GUARD_CLOSED] = True
        self.discrete[DI_UPSTREAM_RDY] = True
        self.discrete[DI_DOWNSTREAM_RDY] = True
        self.discrete[DI_AIR_OK] = True
        # setpoint defaults
        self.holding[HR_SPEED_SP] = 75
        self.holding[HR_TARGET_TPM] = 120
        self.holding[HR_ACCEL_MS] = 2500
        self.holding[HR_DECEL_MS] = 1500
        self.holding[HR_BATCH_SIZE] = 500
        self.holding[HR_DEFECT_PERMILLE] = 20
        self.holding[HR_TEMP_WARN] = TEMP_WARN_DEFAULT_C
        self.holding[HR_TEMP_TRIP] = TEMP_CRIT_DEFAULT_C

    # ---- bank accessors used by the protocol handlers ----
    def read_coils(self, addr, count):
        with self.lock:
            return self.coils[addr:addr + count]

    def read_discrete(self, addr, count):
        with self.lock:
            return self.discrete[addr:addr + count]

    def read_holding(self, addr, count):
        with self.lock:
            return self.holding[addr:addr + count]

    def read_input(self, addr, count):
        with self.lock:
            return self.input_regs[addr:addr + count]

    def write_coil(self, addr, value, operator=True):
        with self.lock:
            # PLC-driven beacons are read-only to outside writers
            if operator and addr not in OPERATOR_COILS and addr in (
                    COIL_BEACON_GREEN, COIL_BEACON_AMBER, COIL_BEACON_RED):
                return False
            self.coils[addr] = bool(value)
            return True

    def write_holding(self, addr, value):
        with self.lock:
            self.holding[addr] = value & 0xFFFF
            return True

    # ---- speed command helper -------------------------------------------
    def set_speed_setpoint(self, pct):
        """Command a new belt speed, clamped to 0-100 %.  Used by the HMI's
        /api/speed endpoint.  (A raw Modbus write to HR0 is deliberately NOT
        clamped here -- an unvalidated write is realistic, and step() limits
        the value it actually acts on.)"""
        with self.lock:
            sp = int(clamp(int(pct), SPEED_MIN_PCT, SPEED_MAX_PCT))
            self.holding[HR_SPEED_SP] = sp
            return sp

    # ---- process simulation -------------------------------------------
    def step(self, dt):
        """Advance the physical model by dt seconds and republish the
        scaled values into the input-register / discrete-input banks."""
        with self.lock:
            self._sim_time += dt
            c = self.coils
            d = self.discrete
            h = self.holding

            # --- handle momentary fault-reset coil ---
            if c[COIL_FAULT_RESET]:
                # the winding has to actually cool before a reset is accepted
                if self.motor_temp < (h[HR_TEMP_TRIP] - TEMP_RESET_MARGIN_C):
                    self.fault_overload = False
                    self.trip_reason = ""
                c[COIL_FAULT_RESET] = False  # spring-return

            # --- safety interlocks (PLC logic) ---
            estop = d[DI_ESTOP]
            guard_ok = d[DI_GUARD_CLOSED]
            safe = (not estop) and guard_ok and (not self.fault_overload)

            run_cmd = c[COIL_RUN] and safe
            if not safe:
                c[COIL_RUN] = c[COIL_RUN] and False  # drop run on trip

            # --- speed ramp via VFD ---
            sp = clamp(h[HR_SPEED_SP], SPEED_MIN_PCT, SPEED_MAX_PCT)
            if 0 < sp < MIN_RUN_SPEED_PCT:
                sp = MIN_RUN_SPEED_PCT      # drive minimum-speed limit
            target = sp if run_cmd else 0.0
            accel = max(h[HR_ACCEL_MS], 100) / 1000.0
            decel = max(h[HR_DECEL_MS], 100) / 1000.0
            if target > self.speed:
                self.speed = min(target, self.speed + (100.0 / accel) * dt)
            elif target < self.speed:
                self.speed = max(target, self.speed - (100.0 / decel) * dt)
            running = self.speed > 1.0

            # --- motor current ---
            if running:
                load = self.speed / 100.0
                base = 2.0 + RATED_CURRENT_A * 0.78 * load
                if d[DI_INFEED_EYE] or d[DI_OUTFEED_EYE]:
                    base += 1.4  # extra load when a part is on the belt
                self.current = base + random.uniform(-0.3, 0.3)
            else:
                self.current = max(0.0, self.current - 4.0 * dt) \
                    + random.uniform(0.0, 0.05)

            # --- motor temperature ---
            fan = c[COIL_COOLING_FAN]
            crit_temp = h[HR_TEMP_TRIP]

            # How far past the continuous rating are we?  0.0 at or below the
            # rating (safe indefinitely), 1.0 at 100 % speed.
            if running and self.speed > CONTINUOUS_SPEED_PCT:
                over = ((self.speed - CONTINUOUS_SPEED_PCT) /
                        (SPEED_MAX_PCT - CONTINUOUS_SPEED_PCT)) ** OVERLOAD_EXP
            else:
                over = 0.0
            self._load_rate = over

            if over > 0.0:
                # Overloaded: heat is going in faster than the frame can shed
                # it, so temperature ramps from wherever the overload started
                # up to the critical limit, accelerating as it climbs.  The
                # ramp is driven by accumulated thermal load, not wall time,
                # so 80 % takes five times as long to get there as 100 % does.
                if self._overload_t0_temp is None:
                    self._overload_t0_temp = self.motor_temp
                self._thermal_load += over * dt
                t0 = self._overload_t0_temp
                top = max(float(crit_temp), t0 + 10.0)
                prog = clamp(self._thermal_load / THERMAL_LOAD_TRIP, 0.0, 1.0)
                ramp = t0 + (top - t0) * (prog ** OVERSPEED_CURVE)
                if fan:
                    ramp -= (ramp - t0) * 0.06   # fan buys a little headroom
                self.motor_temp = max(self.motor_temp, ramp) \
                    + random.uniform(-0.05, 0.05)
            else:
                # At or below the continuous rating: ordinary first-order
                # thermal behaviour toward a steady state that is always well
                # short of the warning level, and accumulated load bleeds off.
                self._thermal_load = max(
                    0.0, self._thermal_load - THERMAL_LOAD_RECOVERY * dt)
                if self._thermal_load <= 0.0:
                    self._overload_t0_temp = None
                tau = (70.0 if fan else 110.0) if running else (55.0 if fan else 95.0)
                heat = (self.current / RATED_CURRENT_A) * 45.0
                target_temp = self.ambient + (heat if running else 0.0)
                self.motor_temp += (target_temp - self.motor_temp) * (dt / tau)

            # stage 1: warning to the operator (5 s at 100 %, later if slower)
            self.warn_overtemp = (
                self._thermal_load >= THERMAL_LOAD_WARN
                or (self.motor_temp >= h[HR_TEMP_WARN] and not self.fault_overload))

            # stage 2: latched critical trip (10 s at 100 %, later if slower)
            if not self.fault_overload:
                over_load = self._thermal_load >= THERMAL_LOAD_TRIP
                over_temp = self.motor_temp >= crit_temp
                if over_load or over_temp:
                    self.fault_overload = True
                    self.motor_temp = max(self.motor_temp, float(crit_temp))
                    self.trip_reason = (
                        "belt held above the %d%% continuous rating -- winding "
                        "reached %d C" % (CONTINUOUS_SPEED_PCT, crit_temp)
                        if over_load else
                        "winding temperature reached %d C" % crit_temp)
                    c[COIL_RUN] = False          # force the conveyor to stop
                    self.warn_overtemp = False
                    self._thermal_load = 0.0
                    self._load_rate = 0.0
                    self._overload_t0_temp = None

            # --- part feeding & travel ---
            if running and self.speed > 5 and d[DI_UPSTREAM_RDY] \
                    and d[DI_DOWNSTREAM_RDY]:
                cap_tpm = (self.speed / 100.0) * 180.0          # belt capacity
                rate_tpm = min(h[HR_TARGET_TPM], cap_tpm)
                self._part_accum += (rate_tpm / 60.0) * dt
                while self._part_accum >= 1.0:
                    self._part_accum -= 1.0
                    self._infeed_timer = 0.30
                    travel = (3.0 * 100.0) / max(self.speed, 1.0)  # belt length/speed
                    defective = random.random() < (h[HR_DEFECT_PERMILLE] / 1000.0)
                    self._in_flight.append([self._sim_time + travel, defective])

            # parts arriving at the outfeed end
            still = []
            for arrive, defective in self._in_flight:
                if self._sim_time >= arrive:
                    self._outfeed_timer = 0.30
                    self.parts += 1
                    self.batch_count += 1
                    self._prod_window.append(self._sim_time)
                    if defective and d[DI_AIR_OK] and not d[DI_REJECT_FULL]:
                        self._kicker_timer = 0.40
                        self.rejects += 1
                    if self.batch_count >= max(h[HR_BATCH_SIZE], 1):
                        self.batch_count = 0  # batch complete, recycle counter
                else:
                    still.append([arrive, defective])
            self._in_flight = still

            # --- throughput over a 60 s rolling window ---
            cutoff = self._sim_time - 60.0
            self._prod_window = [t for t in self._prod_window if t >= cutoff]
            span = min(self._sim_time, 60.0)
            self.throughput = (len(self._prod_window) / span * 60.0) if span > 1 else 0.0

            # --- sensor pulse decay ---
            self._infeed_timer = max(0.0, self._infeed_timer - dt)
            self._outfeed_timer = max(0.0, self._outfeed_timer - dt)
            self._kicker_timer = max(0.0, self._kicker_timer - dt)
            d[DI_INFEED_EYE] = self._infeed_timer > 0
            d[DI_OUTFEED_EYE] = self._outfeed_timer > 0

            # --- vibration ---
            vib = 0.6 + (self.speed / 100.0) * 2.2
            if self.motor_temp > h[HR_TEMP_WARN]:
                vib += 1.0
            self.vibration = max(0.0, vib + random.uniform(-0.15, 0.15))

            # --- electrical & misc ---
            droop = (self.current / RATED_CURRENT_A) * 6.0
            self.line_voltage = 480.0 - droop + random.uniform(-1.5, 1.5)
            self.ambient += random.uniform(-0.02, 0.02)
            self.ambient = clamp(self.ambient, 18.0, 32.0)
            self.scan_ms = 9.0 + random.uniform(0.0, 4.0) + (2.0 if running else 0.0)
            if running:
                self.runtime_s += dt

            # --- write latched-fault sensors ---
            d[DI_MOTOR_OVERLOAD] = self.fault_overload

            # --- stack light (PLC-driven beacons) ---
            warn = self.warn_overtemp or d[DI_REJECT_FULL] \
                or (not d[DI_AIR_OK]) or (not d[DI_DOWNSTREAM_RDY])
            fault = self.fault_overload or estop
            c[COIL_BEACON_RED] = fault
            c[COIL_BEACON_AMBER] = warn and not fault
            c[COIL_BEACON_GREEN] = running and not fault
            # manual kicker test
            if c[COIL_KICKER_TEST]:
                self._kicker_timer = 0.40
                c[COIL_KICKER_TEST] = False

            # --- publish scaled values into input registers ---
            ir = self.input_regs
            ir[IR_BELT_SPEED]    = int(self.speed * SCALE_SPEED) & 0xFFFF
            ir[IR_MOTOR_CURRENT] = int(self.current * 10) & 0xFFFF
            ir[IR_MOTOR_TEMP]    = int(self.motor_temp * SCALE_TEMP) & 0xFFFF
            ir[IR_VFD_FREQ]      = int((self.speed / 100.0) * 60.0 * SCALE_FREQ) & 0xFFFF
            ir[IR_LINE_VOLTAGE]  = int(self.line_voltage) & 0xFFFF
            ir[IR_PARTS_LOW]     = self.parts & 0xFFFF
            ir[IR_PARTS_HIGH]    = (self.parts >> 16) & 0xFFFF
            ir[IR_REJECTS]       = self.rejects & 0xFFFF
            ir[IR_THROUGHPUT]    = int(self.throughput) & 0xFFFF
            ir[IR_RUNTIME_HRS]   = int((self.runtime_s / 3600.0) * SCALE_HRS) & 0xFFFF
            ir[IR_VIBRATION]     = int(self.vibration * SCALE_VIB) & 0xFFFF
            ir[IR_AMBIENT_TEMP]  = int(self.ambient * SCALE_TEMP) & 0xFFFF
            ir[IR_SCAN_MS]       = int(self.scan_ms * SCALE_SCAN) & 0xFFFF
            ir[IR_BATCH_COUNT]   = self.batch_count & 0xFFFF

    # ---- decoded snapshot for the HMI -----------------------------------
    def snapshot(self):
        with self.lock:
            h = self.holding
            d = self.discrete
            c = self.coils
            sp = h[HR_SPEED_SP]
            eff = clamp(sp, SPEED_MIN_PCT, SPEED_MAX_PCT)
            if 0 < eff < MIN_RUN_SPEED_PCT:
                eff = MIN_RUN_SPEED_PCT
            return {
                "ts": time.time(),
                "running": self.speed > 1.0,
                "process": {
                    "belt_speed_pct": round(self.speed, 1),
                    "speed_setpoint_pct": sp,
                    "speed_effective_pct": eff,
                    "speed_min_pct": SPEED_MIN_PCT,
                    "speed_max_pct": SPEED_MAX_PCT,
                    "speed_min_run_pct": MIN_RUN_SPEED_PCT,
                    "accel_ms": h[HR_ACCEL_MS],
                    "decel_ms": h[HR_DECEL_MS],
                    "motor_current_a": round(self.current, 1),
                    "motor_temp_c": round(self.motor_temp, 1),
                    "vfd_freq_hz": round((self.speed / 100.0) * 60.0, 1),
                    "line_voltage_v": round(self.line_voltage, 0),
                    "parts_total": self.parts,
                    "rejects_total": self.rejects,
                    "throughput_tpm": round(self.throughput, 0),
                    "runtime_hrs": round(self.runtime_s / 3600.0, 2),
                    "vibration_mms": round(self.vibration, 2),
                    "ambient_c": round(self.ambient, 1),
                    "scan_ms": round(self.scan_ms, 1),
                    "batch_count": self.batch_count,
                    "batch_size": h[HR_BATCH_SIZE],
                    "temp_warn_c": h[HR_TEMP_WARN],
                    "temp_trip_c": h[HR_TEMP_TRIP],
                    "temp_reset_c": h[HR_TEMP_TRIP] - TEMP_RESET_MARGIN_C,
                    "continuous_pct": CONTINUOUS_SPEED_PCT,
                    "thermal_load": round(self._thermal_load, 2),
                    "load_rate": round(self._load_rate, 3),
                    "load_warn": THERMAL_LOAD_WARN,
                    "load_trip": THERMAL_LOAD_TRIP,
                    "above_rating": self._load_rate > 0.0,
                    "target_tpm": h[HR_TARGET_TPM],
                    "defect_permille": h[HR_DEFECT_PERMILLE],
                },
                "coils": {
                    "run": c[COIL_RUN], "reverse": c[COIL_REVERSE],
                    "cooling_fan": c[COIL_COOLING_FAN],
                    "beacon_green": c[COIL_BEACON_GREEN],
                    "beacon_amber": c[COIL_BEACON_AMBER],
                    "beacon_red": c[COIL_BEACON_RED],
                    "kicker": self._kicker_timer > 0,
                },
                "inputs": {
                    "estop": d[DI_ESTOP], "infeed_eye": d[DI_INFEED_EYE],
                    "outfeed_eye": d[DI_OUTFEED_EYE],
                    "overload": d[DI_MOTOR_OVERLOAD],
                    "guard_closed": d[DI_GUARD_CLOSED],
                    "upstream_rdy": d[DI_UPSTREAM_RDY],
                    "downstream_rdy": d[DI_DOWNSTREAM_RDY],
                    "reject_full": d[DI_REJECT_FULL], "air_ok": d[DI_AIR_OK],
                },
                "faults": {"overload": self.fault_overload,
                           "estop": d[DI_ESTOP],
                           "overtemp_warning": self.warn_overtemp,
                           "trip_reason": self.trip_reason},
            }


# --------------------------------------------------------------------------
# Simulation thread
# --------------------------------------------------------------------------
class Simulator(threading.Thread):
    def __init__(self, model, hz=10.0):
        super().__init__(daemon=True)
        self.model = model
        self.period = 1.0 / hz
        self._stop = threading.Event()

    def run(self):
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now
            self.model.step(dt)
            time.sleep(self.period)

    def stop(self):
        self._stop.set()


# --------------------------------------------------------------------------
# Modbus/TCP server
# --------------------------------------------------------------------------
EXC_ILLEGAL_FUNCTION = 0x01
EXC_ILLEGAL_ADDRESS = 0x02
EXC_ILLEGAL_VALUE = 0x03
EXC_SERVER_FAILURE = 0x04


class ModbusHandler(socketserver.BaseRequestHandler):
    def handle(self):
        model = self.server.model
        sock = self.request
        sock.settimeout(120)
        peer = "%s:%d" % self.client_address
        if self.server.verbose:
            print(f"[modbus] connect {peer}")
        try:
            while True:
                header = self._recv_exact(7)
                if not header:
                    break
                tid, pid, length, unit = struct.unpack(">HHHB", header)
                body = self._recv_exact(length - 1)
                if body is None:
                    break
                resp_pdu = self._dispatch(model, body)
                resp = struct.pack(">HHHB", tid, 0, len(resp_pdu) + 1, unit) + resp_pdu
                sock.sendall(resp)
        except (ConnectionError, socket.timeout, OSError):
            pass
        finally:
            if self.server.verbose:
                print(f"[modbus] disconnect {peer}")

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            try:
                chunk = self.request.recv(n - len(buf))
            except socket.timeout:
                return None
            if not chunk:
                return None if buf == b"" else None
            buf += chunk
        return buf

    @staticmethod
    def _exc(fc, code):
        return struct.pack(">BB", fc | 0x80, code)

    def _dispatch(self, model, pdu):
        if not pdu:
            return self._exc(0, EXC_SERVER_FAILURE)
        fc = pdu[0]
        try:
            if fc in (0x01, 0x02):                       # read bits
                addr, qty = struct.unpack(">HH", pdu[1:5])
                if qty < 1 or qty > 2000 or addr + qty > BANK_SIZE:
                    return self._exc(fc, EXC_ILLEGAL_ADDRESS)
                bits = (model.read_coils if fc == 0x01
                        else model.read_discrete)(addr, qty)
                nbytes = (qty + 7) // 8
                data = bytearray(nbytes)
                for i, b in enumerate(bits):
                    if b:
                        data[i // 8] |= 1 << (i % 8)
                return struct.pack(">BB", fc, nbytes) + bytes(data)

            if fc in (0x03, 0x04):                       # read registers
                addr, qty = struct.unpack(">HH", pdu[1:5])
                if qty < 1 or qty > 125 or addr + qty > BANK_SIZE:
                    return self._exc(fc, EXC_ILLEGAL_ADDRESS)
                regs = (model.read_holding if fc == 0x03
                        else model.read_input)(addr, qty)
                return (struct.pack(">BB", fc, qty * 2)
                        + b"".join(struct.pack(">H", r & 0xFFFF) for r in regs))

            if fc == 0x05:                               # write single coil
                addr, val = struct.unpack(">HH", pdu[1:5])
                if addr >= BANK_SIZE:
                    return self._exc(fc, EXC_ILLEGAL_ADDRESS)
                if val not in (0x0000, 0xFF00):
                    return self._exc(fc, EXC_ILLEGAL_VALUE)
                model.write_coil(addr, val == 0xFF00, operator=True)
                return pdu[:5]

            if fc == 0x06:                               # write single register
                addr, val = struct.unpack(">HH", pdu[1:5])
                if addr >= BANK_SIZE:
                    return self._exc(fc, EXC_ILLEGAL_ADDRESS)
                model.write_holding(addr, val)
                return pdu[:5]

            if fc == 0x0F:                               # write multiple coils
                addr, qty, nbytes = struct.unpack(">HHB", pdu[1:6])
                if addr + qty > BANK_SIZE or qty < 1:
                    return self._exc(fc, EXC_ILLEGAL_ADDRESS)
                data = pdu[6:6 + nbytes]
                for i in range(qty):
                    bit = bool(data[i // 8] & (1 << (i % 8)))
                    model.write_coil(addr + i, bit, operator=True)
                return struct.pack(">BHH", fc, addr, qty)

            if fc == 0x10:                               # write multiple registers
                addr, qty, nbytes = struct.unpack(">HHB", pdu[1:6])
                if addr + qty > BANK_SIZE or qty < 1:
                    return self._exc(fc, EXC_ILLEGAL_ADDRESS)
                data = pdu[6:6 + nbytes]
                for i in range(qty):
                    (val,) = struct.unpack(">H", data[i * 2:i * 2 + 2])
                    model.write_holding(addr + i, val)
                return struct.pack(">BHH", fc, addr, qty)

            return self._exc(fc, EXC_ILLEGAL_FUNCTION)
        except (struct.error, IndexError):
            return self._exc(fc, EXC_ILLEGAL_VALUE)


class ModbusServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, addr, model, verbose=False):
        super().__init__(addr, ModbusHandler)
        self.model = model
        self.verbose = verbose


# --------------------------------------------------------------------------
# Web HMI + JSON API
# --------------------------------------------------------------------------
class HMIHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        if getattr(self.server, "verbose", False):
            super().log_message(*args)

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, HMI_HTML, "text/html; charset=utf-8")
        elif self.path == "/api/state":
            self._send(200, json.dumps(self.server.model.snapshot()))
        elif self.path == "/api/map":
            self._send(200, json.dumps(REGISTER_MAP))
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw or b"{}")
        except json.JSONDecodeError:
            return self._send(400, json.dumps({"error": "bad json"}))
        model = self.server.model

        if self.path == "/api/coil":
            addr, val = payload.get("addr"), bool(payload.get("value"))
            if not isinstance(addr, int) or not (0 <= addr < BANK_SIZE):
                return self._send(400, json.dumps({"error": "bad addr"}))
            ok = model.write_coil(addr, val, operator=True)
            return self._send(200, json.dumps({"ok": ok, "addr": addr, "value": val}))

        if self.path == "/api/speed":
            # dedicated belt-speed command used by the HMI speed station.
            # Accepts either an absolute value or a relative nudge.
            val = payload.get("value")
            delta = payload.get("delta")
            if isinstance(delta, (int, float)):
                with model.lock:
                    val = model.holding[HR_SPEED_SP] + int(delta)
            if not isinstance(val, (int, float)):
                return self._send(400, json.dumps({"error": "bad value"}))
            sp = model.set_speed_setpoint(val)
            return self._send(200, json.dumps({"ok": True, "speed_setpoint_pct": sp}))

        if self.path == "/api/register":
            addr, val = payload.get("addr"), payload.get("value")
            if not isinstance(addr, int) or not (0 <= addr < BANK_SIZE) \
                    or not isinstance(val, int):
                return self._send(400, json.dumps({"error": "bad request"}))
            model.write_holding(addr, val)
            return self._send(200, json.dumps({"ok": True}))

        if self.path == "/api/input":
            # let the HMI flip field sensors (E-stop, guard, etc.)
            addr, val = payload.get("addr"), bool(payload.get("value"))
            if not isinstance(addr, int) or not (0 <= addr < BANK_SIZE):
                return self._send(400, json.dumps({"error": "bad addr"}))
            with model.lock:
                model.discrete[addr] = val
                if addr == DI_MOTOR_OVERLOAD:
                    model.fault_overload = val
            return self._send(200, json.dumps({"ok": True}))

        self._send(404, json.dumps({"error": "not found"}))


class HMIServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, model, verbose=False):
        super().__init__(addr, HMIHandler)
        self.model = model
        self.verbose = verbose


REGISTER_MAP = {
    "coils": {
        "0": "RUN (operator)", "1": "REVERSE (operator)",
        "2": "(reserved)", "3": "COOLING_FAN (operator)",
        "4": "(reserved)", "5": "(reserved)",
        "6": "FAULT_RESET momentary (operator)",
        "7": "KICKER_TEST momentary (operator)",
        "8": "BEACON_GREEN (PLC)", "9": "BEACON_AMBER (PLC)",
        "10": "BEACON_RED (PLC)",
    },
    "discrete_inputs": {
        "0": "ESTOP active", "1": "INFEED_EYE", "2": "OUTFEED_EYE",
        "3": "MOTOR_OVERLOAD", "4": "(reserved)", "5": "GUARD_CLOSED",
        "6": "UPSTREAM_READY", "7": "DOWNSTREAM_READY",
        "8": "REJECT_BIN_FULL", "9": "AIR_PRESSURE_OK",
    },
    "holding_registers": {
        "0": "SPEED_SETPOINT % (0-100)", "1": "TARGET_THROUGHPUT parts/min",
        "2": "ACCEL_TIME ms", "3": "DECEL_TIME ms", "4": "BATCH_SIZE",
        "5": "DEFECT_RATE permille", "6": "TEMP_WARN C",
        "7": "TEMP_CRITICAL_TRIP C (latched, stops conveyor)",
    },
    "input_registers": {
        "0": "BELT_SPEED %x10", "1": "MOTOR_CURRENT Ax10", "2": "MOTOR_TEMP Cx10",
        "3": "VFD_FREQ Hzx10", "4": "LINE_VOLTAGE V", "5": "PARTS_TOTAL low",
        "6": "PARTS_TOTAL high", "7": "REJECTS_TOTAL", "8": "THROUGHPUT parts/min",
        "9": "RUNTIME hoursx10", "10": "VIBRATION mm/s x100", "11": "AMBIENT Cx10",
        "12": "SCAN_TIME ms x10", "13": "BATCH_COUNT",
    },
}

# HMI front-end lives in a separate file string, assembled below.
HMI_HTML = ""  # filled in by _load_hmi() at import time


def _load_hmi():
    global HMI_HTML
    HMI_HTML = HMI_PAGE


# --------------------------------------------------------------------------
# HMI web page  (served from "/")
# --------------------------------------------------------------------------
HMI_PAGE = r"""<!doctype html>
<html lang="en" data-theme="dark">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Zone 1 Conveyor - HMI</title>
<script>
// Apply the stored panel theme before first paint (falls back to the OS setting).
(function(){
  try{
    var t=localStorage.getItem("hmi-theme");
    if(t!=="light"&&t!=="dark"){
      t=(window.matchMedia&&window.matchMedia("(prefers-color-scheme: light)").matches)
        ?"light":"dark";
    }
    document.documentElement.setAttribute("data-theme",t);
  }catch(e){}
})();
</script>
<style>
  :root{
    --mono:ui-monospace,"SF Mono",Menlo,Consolas,"Liberation Mono",monospace;
    --sans:"Inter",system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  }
  /* ---------- dark panel (default) ---------- */
  :root{
    color-scheme:dark;
    --bg:#0f1216; --bg-glow:#1a2029;
    --panel:#171b21; --panel2:#1d232b; --bezel:#283039;
    --bezel-hi:#3a4654; --line:#283039;
    --ink:#b7c0cc; --head:#e7edf4; --dim:#717b87; --off:#3a4654;
    --well:#0c1014; --divider:#ffffff08; --inset-hi:#ffffff08;
    --shadow:0 14px 40px -28px #000;
    --phos:#2cf0a8; --phos-dim:#155c45; --unit:#155c45;
    --glow-phos:#2cf0a855; --glow-amber:#ffb02e55; --glow-red:#ff525255;
    --amber:#ffb02e; --red:#ff5252; --green:#36e07a; --cyan:#46c8ff;
    --btn-top:#222a33; --btn-bot:#1a212a;
    --run-top:#0f3a28; --run-bot:#0c2a1e; --run-line:#1d6b4a; --run-ink:#bff5d9;
    --runon-top:#1d8a55; --runon-bot:#147a47; --runon-line:#3fe089; --runon-ink:#04130b;
    --stop-top:#3a1417; --stop-bot:#2a0f12; --stop-line:#7a2530; --stop-ink:#ffc9c9;
    --estop-ring:#1a0709;
    --lamp-off:#222a32; --sw-off:#222a33; --sw-knob:#556666;
    --sw-red-bg:#5e1a1a; --sw-red-line:#aa3333;
    --alarm-top:#2a1417; --alarm-bot:#1c0f12; --alarm-ink:#ffd5d5;
    --chute:#141a21; --mimic-line:#2a323d; --mimic-deep:#10151b;
    --belt-bg:#0c1117; --belt-chev:#2a3a33;
    --roller:#1c2430; --roller-mark:#5a6878; --sensor:#39424d;
    --part:#caa15a; --part-line:#7a5e25; --part-reject:#ff7a7a;
    --motor-off:#3a4654; --kicker:#5a6878;
  }
  /* ---------- light panel ---------- */
  :root[data-theme="light"]{
    color-scheme:light;
    --bg:#e9edf2; --bg-glow:#ffffff;
    --panel:#ffffff; --panel2:#f6f8fb; --bezel:#c9d2dd;
    --bezel-hi:#94a2b3; --line:#d7dee7;
    --ink:#2a333d; --head:#0f1720; --dim:#5f6a77; --off:#aab4c1;
    --well:#f1f4f8; --divider:#0f172014; --inset-hi:#ffffff;
    --shadow:0 10px 30px -22px #4a5a6e;
    --phos:#0a6f52; --phos-dim:#bfe6d7; --unit:#5d9c85;
    --glow-phos:transparent; --glow-amber:transparent; --glow-red:transparent;
    --amber:#a86a00; --red:#c62f2f; --green:#12864a; --cyan:#0a6fae;
    --btn-top:#ffffff; --btn-bot:#e9edf3;
    --run-top:#d8f2e4; --run-bot:#c2e8d4; --run-line:#5fbf93; --run-ink:#0a5238;
    --runon-top:#1d9e63; --runon-bot:#12854f; --runon-line:#0d6b40; --runon-ink:#ffffff;
    --stop-top:#fde3e3; --stop-bot:#f8d2d2; --stop-line:#e09a9a; --stop-ink:#8a1f1f;
    --estop-ring:#d3dae2;
    --lamp-off:#dbe1e9; --sw-off:#dce3ec; --sw-knob:#94a2b3;
    --sw-red-bg:#fadada; --sw-red-line:#d98a8a;
    --alarm-top:#fdeceb; --alarm-bot:#fff6f5; --alarm-ink:#8a1f1f;
    --chute:#eef2f7; --mimic-line:#c3ccd8; --mimic-deep:#e8edf3;
    --belt-bg:#dfe6ee; --belt-chev:#9dbcae;
    --roller:#e4eaf1; --roller-mark:#8593a3; --sensor:#aab4c1;
    --part:#c99a45; --part-line:#8a6a24; --part-reject:#d94f4f;
    --motor-off:#a4b0be; --kicker:#8e9aa8;
  }
  *{box-sizing:border-box}
  html,body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--sans);
    -webkit-font-smoothing:antialiased}
  body{background:
    radial-gradient(1200px 600px at 50% -10%,var(--bg-glow) 0%,var(--bg) 60%);
    min-height:100vh;padding:0 0 40px}
  .wrap{max-width:1180px;margin:0 auto;padding:0 16px}
  .label{font-size:10.5px;letter-spacing:.18em;text-transform:uppercase;color:var(--dim)}
  .mono{font-family:var(--mono)}

  /* ---------- header ---------- */
  header{display:flex;align-items:center;gap:18px;padding:16px 0 14px;
    border-bottom:1px solid var(--line);margin-bottom:18px}
  .brand{display:flex;flex-direction:column;gap:3px}
  .brand h1{margin:0;font-size:16px;letter-spacing:.04em;font-weight:600;color:var(--head)}
  .brand .sub{font-size:11px;letter-spacing:.22em;text-transform:uppercase;color:var(--dim)}
  .hgap{flex:1}
  .clock{font-family:var(--mono);font-size:13px;color:var(--ink);text-align:right}
  .clock .d{color:var(--dim);font-size:11px}
  .link-state{display:flex;align-items:center;gap:8px;font-size:11px;
    letter-spacing:.14em;text-transform:uppercase;color:var(--dim)}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--dim);
    box-shadow:0 0 0 0 transparent}
  .dot.ok{background:var(--green);box-shadow:0 0 10px var(--green)}
  .dot.bad{background:var(--red);box-shadow:0 0 10px var(--red)}
  button.theme{flex:0 0 auto;background:var(--well);padding:8px 12px;
    font-size:10px;letter-spacing:.16em;color:var(--dim)}
  button.theme:hover{color:var(--ink)}

  /* stack light */
  .stack{display:flex;gap:6px;align-items:center;padding:5px 10px;border-radius:8px;
    background:var(--panel);border:1px solid var(--bezel)}
  .lamp{width:16px;height:16px;border-radius:50%;border:1px solid #0007;
    background:var(--lamp-off);transition:.15s}
  .lamp.g.on{background:var(--green);box-shadow:0 0 14px var(--green)}
  .lamp.a.on{background:var(--amber);box-shadow:0 0 14px var(--amber)}
  .lamp.r.on{background:var(--red);box-shadow:0 0 14px var(--red)}

  /* ---------- alarm banner ---------- */
  #alarm{display:none;margin-bottom:16px;border:1px solid var(--red);
    background:linear-gradient(180deg,var(--alarm-top),var(--alarm-bot));border-radius:10px;
    padding:11px 14px;color:var(--alarm-ink);font-size:13px;letter-spacing:.03em;
    align-items:center;gap:10px}
  #alarm.show{display:flex}
  #alarm.warn{border-color:var(--amber);color:var(--ink)}
  #alarm.warn .tag{color:var(--amber)}
  #alarm.crit{animation:alarmflash 1.1s infinite}
  @keyframes alarmflash{50%{box-shadow:0 0 22px -4px var(--red)}}
  #alarm .tag{font-family:var(--mono);font-weight:700;color:var(--red);
    letter-spacing:.12em;text-transform:uppercase;font-size:11px}

  /* ---------- layout ---------- */
  .grid{display:grid;grid-template-columns:1.55fr 1fr;gap:18px;align-items:start}
  .card{background:linear-gradient(180deg,var(--panel2),var(--panel));
    border:1px solid var(--bezel);border-radius:14px;padding:16px 16px 18px;
    box-shadow:0 1px 0 var(--inset-hi) inset,var(--shadow)}
  .card h2{margin:0 0 14px;font-size:11px;letter-spacing:.2em;text-transform:uppercase;
    color:var(--dim);font-weight:600;display:flex;align-items:center;gap:8px}
  .card h2::before{content:"";width:6px;height:6px;border-radius:1px;
    background:var(--cyan);box-shadow:0 0 8px var(--cyan)}

  /* ---------- mimic (signature) ---------- */
  .mimic{padding:6px}
  svg{display:block;width:100%;height:auto}
  .belt-frame{fill:var(--belt-bg);stroke:var(--bezel-hi);stroke-width:2;rx:6}
  text.lab{fill:var(--dim);font-family:var(--mono);font-size:10px;letter-spacing:.1em}

  /* ---------- instrument readouts ---------- */
  .readouts{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-top:14px}
  .ro{background:var(--well);border:1px solid var(--bezel);border-radius:9px;padding:9px 10px}
  .ro .k{font-size:9.5px;letter-spacing:.13em;text-transform:uppercase;color:var(--dim)}
  .ro .v{font-family:var(--mono);font-size:21px;color:var(--phos);line-height:1.15;
    text-shadow:0 0 12px var(--glow-phos);margin-top:3px}
  .ro .v small{font-size:11px;color:var(--unit);margin-left:3px;text-shadow:none}
  .ro.warn .v{color:var(--amber);text-shadow:0 0 12px var(--glow-amber)}
  .ro.alarm .v{color:var(--red);text-shadow:0 0 12px var(--glow-red)}
  .ro.zero .v{color:var(--off);text-shadow:none}

  /* ---------- controls ---------- */
  .stack-col{display:flex;flex-direction:column;gap:18px}
  .row{display:flex;gap:10px;flex-wrap:wrap}
  button{font-family:var(--sans);cursor:pointer;border:1px solid var(--bezel);
    background:linear-gradient(180deg,var(--btn-top),var(--btn-bot));color:var(--ink);
    border-radius:9px;padding:11px 14px;font-size:12px;letter-spacing:.06em;
    text-transform:uppercase;transition:.12s;flex:1;min-width:0}
  button:hover{border-color:var(--bezel-hi)}
  button:active{transform:translateY(1px)}
  button.primary{background:linear-gradient(180deg,var(--run-top),var(--run-bot));
    border-color:var(--run-line);color:var(--run-ink)}
  button.primary.on{background:linear-gradient(180deg,var(--runon-top),var(--runon-bot));
    border-color:var(--runon-line);color:var(--runon-ink);font-weight:700;
    box-shadow:0 0 18px -4px var(--green)}
  button.stop{background:linear-gradient(180deg,var(--stop-top),var(--stop-bot));
    border-color:var(--stop-line);color:var(--stop-ink)}
  button.estop{background:radial-gradient(circle at 50% 35%,#ff5b5b,#a31616);
    border:2px solid #5e0c0c;color:#fff;font-weight:800;border-radius:50%;
    width:84px;height:84px;flex:0 0 auto;letter-spacing:.04em;line-height:1.1;
    box-shadow:0 0 0 5px var(--estop-ring),0 10px 24px -12px #000}
  button.estop.armed{background:radial-gradient(circle at 50% 35%,#ff8a8a,#c01d1d);
    animation:pulse 1s infinite}
  @keyframes pulse{50%{box-shadow:0 0 0 5px var(--estop-ring),0 0 26px 2px #ff5252}}
  .tgl{display:flex;align-items:center;justify-content:space-between;gap:10px;
    background:var(--well);border:1px solid var(--bezel);border-radius:9px;
    padding:9px 12px;font-size:11.5px;letter-spacing:.05em}
  .sw{width:42px;height:23px;border-radius:20px;background:var(--sw-off);
    border:1px solid var(--bezel);position:relative;cursor:pointer;flex:0 0 auto;
    transition:.15s}
  .sw::after{content:"";position:absolute;top:2px;left:2px;width:17px;height:17px;
    border-radius:50%;background:var(--sw-knob);transition:.15s}
  .sw.on{background:var(--phos-dim);border-color:var(--run-line)}
  .sw.on::after{left:21px;background:var(--green);box-shadow:0 0 8px var(--green)}
  .sw.red.on{background:var(--sw-red-bg);border-color:var(--sw-red-line)}
  .sw.red.on::after{background:var(--red);box-shadow:0 0 8px var(--red)}

  /* ---------- speed command station ---------- */
  .speedbox{margin-top:4px;background:var(--well);border:1px solid var(--bezel);
    border-radius:11px;padding:12px}
  .speedbox .top{display:flex;justify-content:space-between;align-items:baseline;gap:10px}
  .speedbox .act{font-size:10px;letter-spacing:.14em;text-transform:uppercase;
    color:var(--dim)}
  .speedbox .act b{font-family:var(--mono);font-size:16px;color:var(--phos);
    text-shadow:0 0 12px var(--glow-phos);letter-spacing:0;margin-left:5px}
  .spin{display:flex;align-items:stretch;gap:6px;margin-top:10px}
  .spin input[type=number]{flex:1;width:auto;text-align:center;font-size:24px;
    padding:4px 2px;letter-spacing:.02em}
  .presets{display:flex;gap:6px;margin-top:10px}
  button.mini{flex:1;padding:7px 3px;font-size:10px;letter-spacing:.04em;
    border-radius:7px;text-transform:none}
  .spin button.mini{flex:0 0 42px;font-family:var(--mono);font-size:13px;padding:0}
  button.mini.sel{border-color:var(--run-line);color:var(--run-ink);
    background:linear-gradient(180deg,var(--run-top),var(--run-bot))}
  .speedbar{height:5px;border-radius:4px;background:var(--bezel);margin-top:10px;
    overflow:hidden}
  .speedbar{position:relative}
  .speedbar i{display:block;height:100%;width:0;background:var(--phos);
    box-shadow:0 0 10px var(--glow-phos);transition:width .18s linear}
  .speedbar i.over{background:var(--amber);box-shadow:0 0 10px var(--glow-amber)}
  .speedbar u{position:absolute;top:-3px;left:75%;width:2px;height:11px;
    background:var(--bezel-hi);border-radius:1px}
  .hint{font-family:var(--mono);font-size:10px;color:var(--dim);margin-top:9px;
    letter-spacing:.02em}
  .dwell{display:none;margin-top:10px}
  .dwell.show{display:block}
  .dwellbar{position:relative;height:7px;border-radius:4px;background:var(--bezel);
    overflow:hidden}
  .dwellbar i{display:block;height:100%;width:0;background:var(--amber);
    box-shadow:0 0 10px var(--glow-amber);transition:width .1s linear}
  .dwellbar i.crit{background:var(--red);box-shadow:0 0 10px var(--glow-red)}
  .dwellbar .mark{position:absolute;top:0;width:2px;height:100%;
    background:var(--panel);opacity:.85}
  .dwelltxt{font-family:var(--mono);font-size:10px;margin-top:6px;color:var(--amber);
    letter-spacing:.02em}
  .dwelltxt.crit{color:var(--red)}

  input[type=range]{width:100%;-webkit-appearance:none;height:6px;border-radius:6px;
    background:linear-gradient(90deg,var(--phos-dim),var(--bezel));margin-top:12px}
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;
    border-radius:50%;background:var(--phos);cursor:pointer;border:2px solid var(--bg);
    box-shadow:0 0 10px var(--glow-phos)}
  input[type=range]::-moz-range-thumb{width:18px;height:18px;border-radius:50%;
    background:var(--phos);cursor:pointer;border:2px solid var(--bg)}
  input[type=number]{font-family:var(--mono);background:var(--well);color:var(--phos);
    border:1px solid var(--bezel);border-radius:7px;padding:6px 8px;width:70px;font-size:13px}
  .field-row{display:flex;justify-content:space-between;align-items:center;
    padding:7px 0;border-bottom:1px solid var(--divider);font-size:11.5px;letter-spacing:.04em}
  .field-row:last-child{border-bottom:0}
  .help{font-family:var(--mono);font-size:10.5px;color:var(--dim);line-height:1.6;
    margin-top:10px}
  .help b{color:var(--cyan)}
  @media(max-width:880px){.grid{grid-template-columns:1fr}.readouts{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div class="brand">
      <h1>Conveyor Line &mdash; Zone 1</h1>
      <span class="sub">PLC-01 &middot; Modbus/TCP Unit 1</span>
    </div>
    <div class="stack" title="Stack light">
      <div class="lamp r" id="lampR"></div>
      <div class="lamp a" id="lampA"></div>
      <div class="lamp g" id="lampG"></div>
    </div>
    <div class="hgap"></div>
    <button class="theme" id="btnTheme"><span id="themeTxt">Light panel</span></button>
    <div class="link-state"><span class="dot" id="link"></span><span id="linkTxt">connecting</span></div>
    <div class="clock"><div id="clk">--:--:--</div><div class="d" id="clkd">scan -- ms</div></div>
  </header>

  <div id="alarm"><span class="tag" id="alarmTag">Alarm</span><span id="alarmTxt"></span></div>

  <div class="grid">
    <!-- LEFT: mimic + readouts -->
    <div class="card mimic">
      <h2>Process Mimic</h2>
      <svg viewBox="0 0 640 300" id="svg" aria-label="conveyor mimic">
        <!-- infeed chute -->
        <rect x="20" y="120" width="70" height="60" rx="4" fill="var(--chute)" stroke="var(--mimic-line)"/>
        <text class="lab" x="26" y="112">INFEED</text>
        <!-- belt frame -->
        <rect x="96" y="150" width="430" height="42" rx="6" fill="var(--belt-bg)" stroke="var(--bezel-hi)" stroke-width="2"/>
        <!-- moving belt surface -->
        <clipPath id="beltclip"><rect x="98" y="152" width="426" height="38" rx="4"/></clipPath>
        <g clip-path="url(#beltclip)"><g id="chevrons"></g></g>
        <!-- rollers -->
        <circle cx="112" cy="171" r="13" fill="var(--roller)" stroke="var(--bezel-hi)"/>
        <circle cx="510" cy="171" r="13" fill="var(--roller)" stroke="var(--bezel-hi)"/>
        <line id="roll1" x1="112" y1="171" x2="112" y2="160" stroke="var(--roller-mark)" stroke-width="2"/>
        <line id="roll2" x1="510" y1="171" x2="510" y2="160" stroke="var(--roller-mark)" stroke-width="2"/>
        <!-- parts layer -->
        <g id="parts"></g>
        <!-- infeed photo-eye -->
        <rect x="150" y="128" width="8" height="18" fill="var(--sensor)"/>
        <line id="eyeIn" x1="154" y1="146" x2="154" y2="150" stroke="var(--red)" stroke-width="2" opacity="0"/>
        <text class="lab" x="132" y="124">PE-01</text>
        <!-- outfeed photo-eye -->
        <rect x="470" y="128" width="8" height="18" fill="var(--sensor)"/>
        <line id="eyeOut" x1="474" y1="146" x2="474" y2="150" stroke="var(--red)" stroke-width="2" opacity="0"/>
        <text class="lab" x="452" y="124">PE-02</text>
        <!-- reject kicker + chute -->
        <rect x="430" y="196" width="40" height="12" rx="2" fill="var(--roller)" stroke="var(--bezel-hi)"/>
        <rect id="kicker" x="446" y="196" width="8" height="12" fill="var(--kicker)"/>
        <path d="M430 208 L470 208 L482 250 L418 250 Z" fill="var(--mimic-deep)" stroke="var(--mimic-line)"/>
        <text class="lab" x="428" y="266">REJECT</text>
        <!-- outfeed chute -->
        <rect x="540" y="150" width="80" height="42" rx="4" fill="var(--chute)" stroke="var(--mimic-line)"/>
        <text class="lab" x="548" y="142">OUTFEED</text>
        <!-- motor + VFD -->
        <circle cx="510" cy="240" r="22" fill="var(--mimic-deep)" stroke="var(--bezel-hi)" stroke-width="2" id="motorBody"/>
        <g id="motor"><line x1="510" y1="240" x2="510" y2="224" stroke="var(--cyan)" stroke-width="2.5"/></g>
        <text class="lab" x="492" y="276">M / VFD</text>
        <!-- direction arrow -->
        <text id="dir" class="lab" x="290" y="138" style="fill:var(--cyan);font-size:13px;letter-spacing:.2em">&#9654; FORWARD</text>
        <!-- commanded vs actual speed callout -->
        <text id="spdTag" class="lab" x="96" y="290" style="font-size:11px">SPD --.-% / SP --%</text>
      </svg>

      <div class="readouts" id="readouts"></div>
    </div>

    <!-- RIGHT: controls -->
    <div class="stack-col">
      <div class="card">
        <h2>Conveyor Control</h2>
        <div class="row" style="align-items:center">
          <button class="primary" id="btnRun">Start Conveyor Belt</button>
          <button class="estop" id="btnEstop">E&#8209;STOP</button>
        </div>
        <div class="row" style="margin-top:14px">
          <div class="tgl" style="flex:1">Direction reverse
            <div class="sw" id="swRev" data-coil="1"></div></div>
          <div class="tgl" style="flex:1">Cooling fan
            <div class="sw" id="swFan" data-coil="3"></div></div>
        </div>
        <div class="row" style="margin-top:14px">
          <button class="stop" id="btnReset">Fault Reset</button>
          <button id="btnKick">Test Kicker</button>
        </div>
      </div>

      <div class="card">
        <h2>Speed Command &middot; VFD</h2>
        <div class="speedbox">
          <div class="top">
            <span class="label">Setpoint &nbsp;HR0</span>
            <span class="act">actual<b id="spAct">0.0</b>%</span>
          </div>
          <div class="spin">
            <button class="mini" data-nudge="-5" title="slow down 5 %">&minus;5</button>
            <button class="mini" data-nudge="-1" title="slow down 1 %">&minus;1</button>
            <input type="number" id="spNum" min="0" max="100" step="1" value="75"
                   inputmode="numeric" aria-label="belt speed setpoint percent">
            <button class="mini" data-nudge="1" title="speed up 1 %">+1</button>
            <button class="mini" data-nudge="5" title="speed up 5 %">+5</button>
          </div>
          <input type="range" id="speed" min="0" max="100" step="1" value="75"
                 aria-label="belt speed setpoint slider">
          <div class="speedbar"><i id="spBar"></i><u title="continuous rating 75%"></u></div>
          <div class="presets">
            <button class="mini" data-preset="0">Stop</button>
            <button class="mini" data-preset="15">Jog 15</button>
            <button class="mini" data-preset="40">Slow 40</button>
            <button class="mini" data-preset="75">Rated 75</button>
            <button class="mini" data-preset="100">Max 100</button>
          </div>
          <div class="hint" id="spHint">Drag, type, or nudge &mdash; belt follows the VFD ramp.</div>
          <div id="dwell" class="dwell">
            <div class="dwellbar"><i id="dwellFill"></i>
              <span class="mark warnmark" id="markWarn"></span>
              <span class="mark tripmark" id="markTrip"></span></div>
            <div class="dwelltxt" id="dwellTxt"></div>
          </div>
        </div>
        <div class="field-row" style="margin-top:12px">Accel ramp 0&rarr;100&#37; (ms)
          <input type="number" id="hrAccel" data-reg="2" min="100" max="60000" step="100"></div>
        <div class="field-row">Decel ramp 100&rarr;0&#37; (ms)
          <input type="number" id="hrDecel" data-reg="3" min="100" max="60000" step="100"></div>
      </div>

      <div class="card">
        <h2>Setpoints</h2>
        <div class="field-row">Target throughput (parts/min)
          <input type="number" id="hrTpm" data-reg="1" min="0" max="500"></div>
        <div class="field-row">Batch size
          <input type="number" id="hrBatch" data-reg="4" min="1" max="9999"></div>
        <div class="field-row">Defect rate (&permil;)
          <input type="number" id="hrDef" data-reg="5" min="0" max="1000"></div>
        <div class="field-row">Temp warn (&deg;C)
          <input type="number" id="hrWarn" data-reg="6" min="30" max="150"></div>
        <div class="field-row">Critical temp trip (&deg;C)
          <input type="number" id="hrTrip" data-reg="7" min="30" max="200"></div>
      </div>

      <div class="card">
        <h2>Field I/O &middot; Simulate Sensors</h2>
        <div class="row">
          <div class="tgl" style="flex:1">Guard door closed
            <div class="sw" id="diGuard" data-input="5"></div></div>
          <div class="tgl" style="flex:1">Air pressure OK
            <div class="sw" id="diAir" data-input="9"></div></div>
        </div>
        <div class="row" style="margin-top:10px">
          <div class="tgl" style="flex:1">Upstream ready
            <div class="sw" id="diUp" data-input="6"></div></div>
          <div class="tgl" style="flex:1">Downstream ready
            <div class="sw" id="diDown" data-input="7"></div></div>
        </div>
        <div class="row" style="margin-top:10px">
          <div class="tgl" style="flex:1">Reject bin full
            <div class="sw red" id="diBin" data-input="8"></div></div>
        </div>
        <div class="help">
          Modbus map &mdash; <b>coil 0</b> conveyor run &middot;
          <b>HR0</b> speed % &middot; <b>HR1</b> throughput &middot;
          <b>HR2/HR3</b> accel/decel ms &middot;
          <b>IR0</b> speed &middot; <b>IR1</b> current &middot; <b>IR2</b> temp.
          Full map at <b>/api/map</b>.
        </div>
      </div>
    </div>
  </div>
</div>

<script>
const $=s=>document.querySelector(s);
let state=null, fail=0;

async function post(path,body){
  try{await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)});}catch(e){}
}
const setCoil=(a,v)=>post("/api/coil",{addr:a,value:v});
const setReg =(a,v)=>post("/api/register",{addr:a,value:v});
const setInput=(a,v)=>post("/api/input",{addr:a,value:v});
const setSpeed=v=>post("/api/speed",{value:v});

// ---- panel theme ----
const rootEl=document.documentElement;
function themeLabel(){
  $("#themeTxt").textContent =
    rootEl.getAttribute("data-theme")==="light" ? "Dark panel" : "Light panel";
}
$("#btnTheme").onclick=()=>{
  const next = rootEl.getAttribute("data-theme")==="light" ? "dark" : "light";
  rootEl.setAttribute("data-theme",next);
  try{localStorage.setItem("hmi-theme",next);}catch(e){}
  themeLabel();
};
themeLabel();

// ---- controls ----
$("#btnRun").onclick=()=>{ if(state) setCoil(0,!state.coils.run); };
$("#btnEstop").onclick=()=>{ if(state) setInput(0,!state.inputs.estop); };
$("#btnReset").onclick=()=>setCoil(6,true);
$("#btnKick").onclick=()=>setCoil(7,true);

/* ---------- belt speed command station ----------------------------------
   One local value (spLocal) drives the slider, the numeric box and the bar,
   so they never disagree.  Writes to HR0 are throttled while dragging so a
   fast drag does not flood the PLC, then a final write lands on release.
   Poll-sync is suppressed for a moment after any local edit (spDirty) so the
   control does not snap back before the write is acknowledged.            */
const SP_MIN=0, SP_MAX=100, CONT_PCT=75;   // 75 % = motor continuous rating
let spLocal=75, spDirty=0, spTimer=null, spPending=null;

function flushSpeed(){
  spTimer=null;
  if(spPending!==null){ setSpeed(spPending); spPending=null; }
}
function applySpeed(v,immediate){
  v=Math.round(v);
  if(isNaN(v)) return;
  v=Math.max(SP_MIN,Math.min(SP_MAX,v));
  spLocal=v; spDirty=Date.now();
  paintSpeed(v);
  if(immediate){
    if(spTimer){clearTimeout(spTimer); spTimer=null;}
    spPending=null; setSpeed(v);
  }else{
    spPending=v;
    if(!spTimer) spTimer=setTimeout(flushSpeed,120);
  }
}
function paintSpeed(v){
  const sl=$("#speed"), nu=$("#spNum");
  if(document.activeElement!==sl) sl.value=v;
  if(document.activeElement!==nu) nu.value=v;
  $("#spBar").style.width=v+"%";
  $("#spBar").classList.toggle("over",v>CONT_PCT);
  document.querySelectorAll("button[data-preset]").forEach(b=>{
    b.classList.toggle("sel", +b.dataset.preset===v);
  });
}
$("#speed").addEventListener("input",e=>applySpeed(+e.target.value,false));
$("#speed").addEventListener("change",e=>applySpeed(+e.target.value,true));
$("#spNum").addEventListener("input",e=>{
  const v=parseInt(e.target.value,10);
  if(!isNaN(v)&&v>=SP_MIN&&v<=SP_MAX){ spLocal=v; spDirty=Date.now();
    $("#speed").value=v; $("#spBar").style.width=v+"%"; }
});
$("#spNum").addEventListener("change",e=>applySpeed(parseInt(e.target.value,10),true));
$("#spNum").addEventListener("keydown",e=>{
  if(e.key==="Enter"){ applySpeed(parseInt(e.target.value,10),true); e.target.blur(); }
});
document.querySelectorAll("button[data-nudge]").forEach(b=>{
  b.onclick=()=>applySpeed(spLocal+(+b.dataset.nudge),true);
});
document.querySelectorAll("button[data-preset]").forEach(b=>{
  b.onclick=()=>applySpeed(+b.dataset.preset,true);
});
// keyboard: up/down nudge 1 %, shift+up/down nudge 10 % (ignored while typing)
document.addEventListener("keydown",e=>{
  const t=e.target.tagName;
  if(t==="INPUT"||t==="TEXTAREA") return;
  if(e.key==="ArrowUp"||e.key==="ArrowDown"){
    const step=(e.shiftKey?10:1)*(e.key==="ArrowUp"?1:-1);
    applySpeed(spLocal+step,true); e.preventDefault();
  }
});

document.querySelectorAll(".sw[data-coil]").forEach(sw=>{
  sw.onclick=()=>{ if(!state)return; const a=+sw.dataset.coil;
    const cur=sw.classList.contains("on"); setCoil(a,!cur); };
});
document.querySelectorAll(".sw[data-input]").forEach(sw=>{
  sw.onclick=()=>{ if(!state)return; const a=+sw.dataset.input;
    const cur=sw.classList.contains("on"); setInput(a,!cur); };
});
document.querySelectorAll("input[data-reg]").forEach(inp=>{
  inp.addEventListener("change",()=>setReg(+inp.dataset.reg,parseInt(inp.value)||0));
});

// ---- build belt chevrons ----
const chev=$("#chevrons");
for(let i=0;i<22;i++){
  const x=98+i*22;
  const c=document.createElementNS("http://www.w3.org/2000/svg","path");
  c.setAttribute("d",`M${x} 154 l10 8 l-10 8`);
  c.setAttribute("fill","none");c.setAttribute("stroke","var(--belt-chev)");
  c.setAttribute("stroke-width","2");
  chev.appendChild(c);
}
let beltOff=0;

// ---- parts particle system ----
const partsG=$("#parts");
let parts=[]; // {el,x,reject}
let lastIn=false, lastKick=false;
const X0=120, X1=500, Y=158;
function spawnPart(){
  const r=document.createElementNS("http://www.w3.org/2000/svg","rect");
  r.setAttribute("width","16");r.setAttribute("height","14");
  r.setAttribute("rx","2");r.setAttribute("y",Y);
  r.setAttribute("fill","var(--part)");r.setAttribute("stroke","var(--part-line)");
  partsG.appendChild(r);
  parts.push({el:r,x:X0,reject:false,dropped:false});
}

let prev=performance.now();
function animate(now){
  const dt=(now-prev)/1000; prev=now;
  const sp=state? state.process.belt_speed_pct:0;
  const rev=state&&state.coils.reverse;
  const pxs=(sp/100)*150*(rev?-1:1);
  beltOff=(beltOff + pxs*dt) % 22;
  chev.setAttribute("transform",`translate(${beltOff},0)`);
  // rollers spin
  const ang=(now/1000)*sp*3*(rev?-1:1);
  $("#roll1").setAttribute("transform",`rotate(${ang} 112 171)`);
  $("#roll2").setAttribute("transform",`rotate(${ang} 510 171)`);
  $("#motor").setAttribute("transform",`rotate(${ang*1.6} 510 240)`);
  // move parts
  parts.forEach(p=>{
    if(p.dropped){ p.y=(p.y||196)+120*dt; p.el.setAttribute("y",p.y);
      p.el.setAttribute("x",455); return; }
    p.x += (sp/100)*120*dt*(rev?-1:1);
    p.el.setAttribute("x",p.x);
    if(p.reject && p.x>=448){ p.dropped=true; p.el.setAttribute("fill","var(--part-reject)"); }
  });
  parts=parts.filter(p=>{
    const gone = p.x>X1+40 || p.x<X0-40 || (p.y||0)>250;
    if(gone) p.el.remove();
    return !gone;
  });
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

// ---- render ----
function lamp(id,on){ $(id).classList.toggle("on",!!on); }
function sw(id,on){ const e=$(id); if(e) e.classList.toggle("on",!!on); }

function ro(k,v,unit,cls){
  return `<div class="ro ${cls||''}"><div class="k">${k}</div>`+
    `<div class="v">${v}<small>${unit||''}</small></div></div>`;
}
function render(){
  const p=state.process, c=state.coils, i=state.inputs, f=state.faults;
  // stack light
  lamp("#lampG",c.beacon_green); lamp("#lampA",c.beacon_amber); lamp("#lampR",c.beacon_red);
  // run button
  $("#btnRun").classList.toggle("on",c.run);
  $("#btnRun").textContent=c.run?"Stop Conveyor Belt":"Start Conveyor Belt";
  $("#btnEstop").classList.toggle("armed",i.estop);
  // toggles reflect live coil/input state
  sw("#swRev",c.reverse); sw("#swFan",c.cooling_fan);
  sw("#diGuard",i.guard_closed); sw("#diAir",i.air_ok); sw("#diUp",i.upstream_rdy);
  sw("#diDown",i.downstream_rdy); sw("#diBin",i.reject_full);
  // direction label
  $("#dir").innerHTML=c.reverse?"&#9664; REVERSE":"&#9654; FORWARD";
  // photo-eyes
  $("#eyeIn").setAttribute("opacity",i.infeed_eye?1:0);
  $("#eyeOut").setAttribute("opacity",i.outfeed_eye?1:0);
  $("#motorBody").setAttribute("stroke",p.belt_speed_pct>1?"var(--cyan)":"var(--motor-off)");
  // kicker visual
  $("#kicker").setAttribute("x",c.kicker?452:446);
  $("#kicker").setAttribute("fill",c.kicker?"var(--part-reject)":"var(--kicker)");

  // ---- speed station ----
  $("#spAct").textContent=p.belt_speed_pct.toFixed(1);
  $("#spdTag").textContent="SPD "+p.belt_speed_pct.toFixed(1)+"% / SP "+p.speed_setpoint_pct+"%";
  // adopt the PLC value unless the operator just touched the control
  if(Date.now()-spDirty>900 && document.activeElement!==$("#spNum")
     && document.activeElement!==$("#speed")){
    spLocal=p.speed_setpoint_pct;
    paintSpeed(spLocal);
  }
  const ramping=Math.abs(p.belt_speed_pct-(c.run?p.speed_effective_pct:0))>0.6;
  let hint;
  if(!c.run) hint="Conveyor stopped \u2014 setpoint armed at "+p.speed_setpoint_pct+"%.";
  else if(ramping) hint=(p.belt_speed_pct < p.speed_effective_pct ? "Accelerating" : "Decelerating")
       +" \u2192 "+p.speed_effective_pct+"% ("+(p.belt_speed_pct<p.speed_effective_pct?p.accel_ms:p.decel_ms)+" ms ramp)";
  else if(p.speed_setpoint_pct>0 && p.speed_setpoint_pct<p.speed_min_run_pct)
       hint="Below drive minimum \u2014 held at "+p.speed_min_run_pct+"%.";
  else if(p.above_rating) hint="ABOVE CONTINUOUS RATING ("+p.continuous_pct
       +"%) \u2014 motor is accumulating heat.";
  else hint="At speed \u2014 "+p.vfd_freq_hz.toFixed(1)+" Hz to the motor.";
  $("#spHint").textContent=hint;

  // ---- thermal load meter: warning at 5, critical trip at 10 ----
  const dw=$("#dwell");
  if(p.above_rating||p.thermal_load>0){
    const L=p.thermal_load, W=p.load_warn, T=p.load_trip, rate=p.load_rate;
    const past=L>=W;
    const secs=x=>rate>0 ? (x/rate).toFixed(0)+" s" : "\u2014";
    dw.classList.add("show");
    $("#dwellFill").style.width=Math.min(100,(L/T)*100)+"%";
    $("#dwellFill").classList.toggle("crit",past);
    $("#markWarn").style.left=(W/T)*100+"%";
    $("#markTrip").style.left="calc(100% - 2px)";
    $("#dwellTxt").classList.toggle("crit",past);
    $("#dwellTxt").textContent = rate<=0
      ? "Back at or below "+p.continuous_pct+"% \u2014 thermal load "+L.toFixed(1)
        +"/"+T.toFixed(0)+" bleeding off \u00B7 winding "+p.motor_temp_c.toFixed(0)+"\u00B0C"
      : "Thermal load "+L.toFixed(1)+"/"+T.toFixed(0)+" at "
        +p.belt_speed_pct.toFixed(0)+"% \u00B7 "
        +(past ? "OVERTEMP WARNING \u00B7 trip in "+secs(T-L)
               : "warning in "+secs(W-L)+", trip in "+secs(T-L))
        +" \u00B7 winding "+p.motor_temp_c.toFixed(0)+"\u00B0C / "+p.temp_trip_c+"\u00B0C";
  } else {
    dw.classList.remove("show");
    $("#dwellFill").style.width="0%";
  }

  // readouts
  const tw=p.motor_temp_c>=p.temp_warn_c||f.overtemp_warning,
        tt=p.motor_temp_c>=p.temp_trip_c||f.overload;
  $("#readouts").innerHTML=
    ro("Belt speed",p.belt_speed_pct.toFixed(1),"%",p.belt_speed_pct<1?'zero':'')+
    ro("Speed setpt",p.speed_setpoint_pct,"%",p.speed_setpoint_pct<1?'zero':'')+
    ro("VFD freq",p.vfd_freq_hz.toFixed(1),"Hz",p.vfd_freq_hz<1?'zero':'')+
    ro("Motor current",p.motor_current_a.toFixed(1),"A")+
    ro("Motor temp",p.motor_temp_c.toFixed(1),"\u00B0C",tt?'alarm':tw?'warn':'')+
    ro("Throughput",p.throughput_tpm,"ppm")+
    ro("Parts total",p.parts_total,"")+
    ro("Rejects",p.rejects_total,"",p.rejects_total>0?'warn':'')+
    ro("Vibration",p.vibration_mms.toFixed(2),"mm/s",p.vibration_mms>5?'alarm':'')+
    ro("Line voltage",p.line_voltage_v,"V")+
    ro("Runtime",p.runtime_hrs.toFixed(2),"h")+
    ro("Batch",p.batch_count+"/"+p.batch_size,"");
  // sync inputs that aren't being edited
  const sync=(id,val)=>{const e=$(id); if(e&&document.activeElement!==e)e.value=val;};
  sync("#hrTpm",p.target_tpm); sync("#hrBatch",p.batch_size);
  sync("#hrDef",p.defect_permille); sync("#hrWarn",p.temp_warn_c);
  sync("#hrTrip",p.temp_trip_c);
  sync("#hrAccel",p.accel_ms); sync("#hrDecel",p.decel_ms);
  // ---- alarm banner: critical (red) outranks warning (amber) ----
  const crit=[], warn=[];
  if(f.estop) crit.push("EMERGENCY STOP ACTIVE");
  if(f.overload) crit.push("CRITICAL MOTOR OVERTEMPERATURE \u2014 conveyor tripped"
      +(f.trip_reason?" ("+f.trip_reason+")":"")
      +" \u2014 cool below "+p.temp_reset_c+"\u00B0C, then Fault Reset");
  if(!i.guard_closed) crit.push("GUARD DOOR OPEN");
  if(f.overtemp_warning) warn.push("MOTOR OVERTEMP WARNING \u2014 "
      +p.motor_temp_c.toFixed(0)+"\u00B0C and climbing \u2014 REDUCE BELT SPEED TO "
      +p.continuous_pct+"% OR BELOW"
      +(p.load_rate>0?" ("+((p.load_trip-p.thermal_load)/p.load_rate).toFixed(0)
        +" s to trip)":""));
  if(i.reject_full) warn.push("REJECT BIN FULL");
  if(!i.air_ok) warn.push("LOW AIR PRESSURE");
  const a=$("#alarm"), all=crit.concat(warn);
  if(all.length){
    a.classList.add("show");
    a.classList.toggle("warn",crit.length===0);
    a.classList.toggle("crit",crit.length>0);
    $("#alarmTag").textContent=crit.length?"Alarm":"Warning";
    $("#alarmTxt").textContent=all.join("   \u2022   ");
  } else { a.classList.remove("show","warn","crit"); }
  // clock
  const d=new Date();
  $("#clk").textContent=d.toLocaleTimeString();
  $("#clkd").textContent="scan "+p.scan_ms.toFixed(1)+" ms";
}

// rising-edge detection for spawning parts, driven by infeed eye
function edges(){
  if(!state) return;
  const inEye=state.inputs.infeed_eye;
  if(inEye && !lastIn) spawnPart();
  lastIn=inEye;
  // mark nearest mid-belt part as reject when kicker fires
  const k=state.coils.kicker;
  if(k && !lastKick){
    let best=null;
    parts.forEach(p=>{ if(!p.dropped && p.x>300 && (!best||p.x>best.x)) best=p; });
    if(best) best.reject=true;
  }
  lastKick=k;
}

async function poll(){
  try{
    const r=await fetch("/api/state",{cache:"no-store"});
    state=await r.json(); fail=0;
    $("#link").className="dot ok"; $("#linkTxt").textContent="online";
    edges(); render();
  }catch(e){
    fail++;
    if(fail>2){$("#link").className="dot bad";$("#linkTxt").textContent="no link";}
  }
}
setInterval(poll,400); poll();
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Simulated manufacturing-line PLC")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--modbus-port", type=int, default=502)
    ap.add_argument("--hmi-port", type=int, default=60000)
    ap.add_argument("--rate", type=float, default=10.0, help="sim ticks per second")
    ap.add_argument("--speed", type=int, default=None,
                    help="initial belt speed setpoint, 0-100 %% (default 75)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    _load_hmi()
    model = PlantModel()
    if args.speed is not None:
        model.set_speed_setpoint(args.speed)
    sim = Simulator(model, hz=args.rate)
    sim.start()

    try:
        mb = ModbusServer((args.host, args.modbus_port), model, args.verbose)
    except PermissionError:
        raise SystemExit(
            f"Permission denied binding port {args.modbus_port}. "
            f"Re-run with --modbus-port 5020 (unprivileged) or as root.")
    hmi = HMIServer((args.host, args.hmi_port), model, args.verbose)

    threading.Thread(target=mb.serve_forever, daemon=True).start()
    threading.Thread(target=hmi.serve_forever, daemon=True).start()

    print("=" * 64)
    print(" Simulated PLC  --  Conveyor Line  Zone 1")
    print("=" * 64)
    print(f" Modbus/TCP : {args.host}:{args.modbus_port}  (unit id 1)")
    print(f" Web HMI    : http://{args.host}:{args.hmi_port}/")
    print(f"   conveyor run  -> coil {COIL_RUN}  (FC05: 0xFF00 on / 0x0000 off)")
    print(f"   speed setpt   -> holding reg {HR_SPEED_SP}  (0-100 %)"
          f"   [now {model.holding[HR_SPEED_SP]} %]")
    print(f"   accel / decel -> holding regs {HR_ACCEL_MS} / {HR_DECEL_MS}  (ms)")
    print("   HMI speed station: slider, numeric entry, +/-1 and +/-5 nudge,")
    print("   presets, and up/down arrow keys (shift = 10 %).")
    print(f" Motor continuous rating: {CONTINUOUS_SPEED_PCT:.0f} % belt speed --"
          f" safe to run indefinitely.")
    print(f"   Above it the motor overloads and trips at "
          f"{model.holding[HR_TEMP_TRIP]} C winding, forcing the conveyor to stop:")
    for pct in (80, 85, 90, 95, 100):
        rate = ((pct - CONTINUOUS_SPEED_PCT) /
                (SPEED_MAX_PCT - CONTINUOUS_SPEED_PCT)) ** OVERLOAD_EXP
        print(f"     {pct:3d} %  warning {THERMAL_LOAD_WARN / rate:5.0f} s   "
              f"trip {THERMAL_LOAD_TRIP / rate:5.0f} s")
    print(" Ctrl+C to stop.")
    print("=" * 64)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nshutting down...")
    finally:
        sim.stop()
        mb.shutdown()
        hmi.shutdown()


if __name__ == "__main__":
    main()