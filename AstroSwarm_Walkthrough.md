# AstroSwarm — Full Code Walkthrough

A beginner-friendly section-by-section explanation of the two controller files that drive the AstroSwarm Mars-mission simulation in Webots.

**Files covered**
- `controllers/beacon_controller/beacon_controller.py` — the beacon (glowing sphere)
- `controllers/astroswarm_controller/astroswarm_controller.py` — the three e-puck rovers

---

## Table of Contents

1. [Big Picture](#big-picture)
2. [How the Beacon Works (Conceptually)](#how-the-beacon-works-conceptually)
3. [How the Robots Search](#how-the-robots-search)
4. [Section 1 — `beacon_controller.py`](#section-1--beacon_controllerpy)
5. [Section 2 — Imports & Hardware Constants](#section-2--imports--hardware-constants)
6. [Section 3 — Sensor Thresholds & Avoidance Timing](#section-3--sensor-thresholds--avoidance-timing)
7. [Section 4 — Per-Robot Configuration & State Labels](#section-4--per-robot-configuration--state-labels)
8. [Section 5 — Robot Initialization](#section-5--robot-initialization)
9. [Section 6 — Motion & Sensor Helpers](#section-6--motion--sensor-helpers)
10. [Section 7 — `classify_obstacle`](#section-7--classify_obstacle)
11. [Section 8 — State Variables & Main Loop Top](#section-8--state-variables--main-loop-top)
12. [Section 9 — NASA Uplink & `INIT_TURN`](#section-9--nasa-uplink--init_turn)
13. [Section 10 — The `DRIVE` State](#section-10--the-drive-state)
14. [Section 11 — `REVERSE`, `TURN`, `REORIENT`](#section-11--reverse-turn-reorient)
15. [Whole-System Summary](#whole-system-summary)

---

## Big Picture

**Webots** is a robot simulator. You build a virtual world (rocks, walls, robots) and write Python scripts called **controllers** that run *inside* each robot. Every robot in the world has its own controller script.

**Project (AstroSwarm) — a Mars mission simulation:**
- 3 little robots called **e-pucks** (a real research robot — small two-wheeled disc with sensors)
- 1 stationary **beacon** (a glowing sphere they need to find)
- An arena with rocks/walls representing Martian terrain

**Goal:** the 3 robots roam, dodge obstacles, and try to detect the beacon. Once any robot finds it, it "calls NASA" (prints fake telemetry) periodically.

**The two files:**
1. `beacon_controller.py` — runs *inside the beacon*. Its only job: shout "BEACON" on a radio channel constantly so robots can hear it.
2. `astroswarm_controller.py` — runs *inside each of the 3 robots*. The brain: drive, sense obstacles, listen for beacon, behave like a state machine.

**Key concept — state machine:** instead of doing everything at once, the robot is always in *one* mode (driving, reversing, turning, etc.) and switches between them based on what it senses.

---

## How the Beacon Works (Conceptually)

The beacon does **not** tell the robot where it is. It only tells the robot *"you are close enough to hear me."*

Think of it like a smoke alarm beeping in a dark house. You can't tell which room it's in by the beep — but if you can hear it at all, you know you're inside the house.

In Webots, the beacon's **Emitter** has `range = 0.5 m`. The signal is a yes/no thing:
- Robot is within 0.5 m → **Receiver** picks up `"BEACON"` → robot knows *"I'm right next to it."*
- Robot is farther → silence → robot has no clue if the beacon even exists.

The beacon is essentially a **proximity confirmation tool**, not a GPS or compass. The robots have to physically wander close to it before the signal ever reaches them.

---

## How the Robots Search

The robots **don't have a map** and they don't search systematically. Each robot just **drives roughly east in a straight line** and only turns when it bumps into something. It's dumb wandering, not smart searching.

The variables aren't random — they're **hardcoded and different per robot**:

| Robot  | Initial turn | Avoidance turn | Default avoid side |
|--------|--------------|---------------|--------------------|
| robot1 | 22.6° left   | 90°           | right              |
| robot2 | 20.0° right  | 105°          | left               |
| robot3 | 2.2° right   | 120°          | right              |

Each robot is **deterministic** (same behavior every run) but **deliberately tuned differently** so they don't move identically. With three slightly different wanderers covering different paths, *one of them* will hopefully stumble close enough to the beacon to hear it.

> **Three deterministic wanderers, each tuned slightly differently, increase the odds that at least one bumbles into the 0.5 m beacon zone.**

**What happens after finding the beacon?** The robot's *driving behavior stays the same*. What changes:
- It logs a fancy `BEACON FOUND` message once.
- It starts a periodic NASA uplink (printing fake telemetry every 10 sim-minutes).
- If it hears the beacon again later, it logs `RE-ESTABLISHING UPLINK`.

The robot doesn't stop, doesn't celebrate beyond the print, and doesn't tell its teammates.

---

## Section 1 — `beacon_controller.py`

The brain of the **glowing sphere** sitting in the arena. Its only job: shout "BEACON" forever.

### Background

**Emitter / Receiver in Webots:** like a walkie-talkie pair.
- An **Emitter** broadcasts messages on a *channel* (e.g. channel 1) with a *range* in metres.
- A **Receiver** tuned to the same channel picks up the message *only if* it's within range.

The range and channel are set on the **Emitter device in the world file** (`.wbt`), not in this Python code.

**Simulation step:** Webots runs in discrete time chunks called **steps** (here, 32 ms each). On every step, every controller gets one chance to read sensors and act.

### The code

```python
"""
Broadcasts a "BEACON" signal on channel 1 every simulation step.
"""

from controller import Robot

robot    = Robot()
emitter  = robot.getDevice("emitter")
TIME_STEP = 32

while robot.step(TIME_STEP) != -1:
    emitter.send("BEACON".encode("utf-8"))
```

| Line | Meaning |
|------|---------|
| `from controller import Robot` | Imports Webots' Python API |
| `robot = Robot()` | Creates the robot object — even though the beacon doesn't move, anything with a controller is a `Robot` |
| `emitter = robot.getDevice("emitter")` | Gets a Python handle to the Emitter device that was attached in the `.wbt` |
| `TIME_STEP = 32` | Tick duration in ms |
| `while robot.step(TIME_STEP) != -1:` | Main loop, runs forever until Webots shuts down |
| `emitter.send("BEACON".encode("utf-8"))` | Convert string to bytes (emitters need bytes, not strings) and broadcast |

**Net effect:** every 32 ms, this beacon yells "BEACON" into the void. Anyone within 0.5 m hears it.

---

## Section 2 — Imports & Hardware Constants

Lines 1–31 of `astroswarm_controller.py`.

### Background

**Radians vs degrees.** Code measures angles in radians.
- 360° = 2π rad ≈ 6.28 rad
- 90° = π/2 rad ≈ 1.57 rad
- The e-puck's `MAX_SPEED = 6.28` rad/s means at full throttle, a wheel spins exactly **one full revolution per second**.

**Differential drive — wheel speed → body rotation.** When two wheels spin in opposite directions, the robot **spins in place**. The math:

```
ω_body = 2 × wheel_speed × wheel_radius / axle_length
```

This converts "how fast the wheels are turning" into "how fast the robot rotates." The code uses it to figure out how many time-steps it needs to spin to achieve a target *angle*.

**Time-step thinking.** Everything happens in 32-ms ticks. To "spin 90°" the code calculates: *"at this rotation speed, how many 32-ms ticks does that take?"* and counts ticks. No gyroscope needed.

### The code

```python
TIME_STEP        = 32                    # ms per Webots step
DT               = TIME_STEP / 1000.0    # 0.032 s
MAX_SPEED        = 6.28                  # rad/s — e-puck motor limit

DRIVE_SPEED      = 5.5                   # cruise forward speed
TURN_SPEED       = 3.0                   # obstacle-avoidance spin speed
INIT_TURN_SPEED  = 2.0                   # initial fine heading correction

WHEEL_RADIUS  = 0.0205                   # m
AXLE_LENGTH   = 0.052                    # m, centre-to-centre

_INIT_OMEGA = 2.0 * INIT_TURN_SPEED * WHEEL_RADIUS / AXLE_LENGTH  # ≈ 1.577 rad/s
_TURN_OMEGA = 2.0 * TURN_SPEED      * WHEEL_RADIUS / AXLE_LENGTH  # ≈ 2.365 rad/s

def _steps_for_angle(degrees, omega):
    return max(1, int(round(math.radians(abs(degrees)) / (omega * DT))))
```

### Three speed presets — when each is used

| Constant            | Value | Used for                                              |
|---------------------|-------|-------------------------------------------------------|
| `DRIVE_SPEED`       | 5.5   | Cruising straight (just under max)                    |
| `TURN_SPEED`        | 3.0   | Big avoidance spins (90–120°), needs to be quick      |
| `INIT_TURN_SPEED`   | 2.0   | Small precise turns (≤ 45°), slow to avoid overshoot |

**Rule of thumb:** big turn → fast speed. Small turn → slow speed.

### `_steps_for_angle` — worked example

Want a 90° turn at `_TURN_OMEGA = 2.365` rad/s?
- 90° = π/2 ≈ 1.571 rad
- per tick: 2.365 × 0.032 ≈ 0.0757 rad
- 1.571 / 0.0757 ≈ 20.75 → rounds to **21 ticks**

So `_steps_for_angle(90, _TURN_OMEGA) = 21`. The robot will spin for 21 ticks (about 672 ms) to achieve a 90° turn.

---

## Section 3 — Sensor Thresholds & Avoidance Timing

Lines 33–67.

### Background

**IR proximity sensors.** The e-puck has 8 sensors arranged around its body. Each shoots an invisible IR beam and measures how much bounces back.
- **Low value (~0)** → nothing close
- **High value (hundreds–thousands)** → something right in front

Higher reading = closer/bigger obstacle.

**Hysteresis (two thresholds, not one).** Like a thermostat that turns on at 19°C and off at 21°C — a "dead zone" prevents flickering. Same trick here:

| Constant       | Value | Meaning                          |
|----------------|-------|----------------------------------|
| `OBSTACLE_TH`  | 80    | Triggers obstacle response       |
| `CLEAR_TH`     | 40    | Considered "open air"            |

### The code

```python
OBSTACLE_TH = 80
CLEAR_TH    = 40

BEACON_SIGNAL    = "BEACON"
NASA_INTERVAL_S  = 600.0    # 10 sim-minutes

REVERSE_STEPS  = 22
REORIENT_STEPS = _steps_for_angle(45, _INIT_OMEGA)   # ≈ 16 steps

_TURN_STEPS = {
    "robot1": _steps_for_angle( 90, _TURN_OMEGA),    # ≈ 21 steps
    "robot2": _steps_for_angle(105, _TURN_OMEGA),    # ≈ 24 steps
    "robot3": _steps_for_angle(120, _TURN_OMEGA),    # ≈ 28 steps
}
```

### Why these numbers

- **`REVERSE_STEPS = 22`** — back up ~700 ms before turning. Spinning while pressed against a wall just grinds the wheels.
- **`REORIENT_STEPS ≈ 16`** — counter-spin ~45° after the big avoidance turn. Partial undo so the robot points roughly east again *without* re-pointing at what it just dodged.
- **Per-robot turn angles (90 / 105 / 120°)** — **anti-oscillation / symmetry-breaking.** If all three robots had identical turns, they could lock into a synchronized loop. Different angles guarantee they desynchronize after collisions.

### Anti-oscillation in plain English

Three identical robots running into each other at the same angle would bounce off in perfect mirror image and re-collide forever. Different turn amounts make them diverge after one collision. This is a real swarm-robotics technique called **symmetry breaking**.

---

## Section 4 — Per-Robot Configuration & State Labels

Lines 69–102.

### Why one big config dict?

All three robots run **the same Python file**. They behave differently because at startup each robot calls `robot.getName()` and looks itself up in this dict. **One file → three personalities.**

### The configuration

```python
ROBOT_CONFIG = {
    "robot1": {
        "init_dir":         "left",
        "init_steps":       _steps_for_angle(22.6, _INIT_OMEGA),  # ≈ 8 steps
        "avoid_default":    "right",
        "avoid_turn_steps": _TURN_STEPS["robot1"],
    },
    "robot2": {
        "init_dir":         "right",
        "init_steps":       _steps_for_angle(20.0, _INIT_OMEGA),  # ≈ 7 steps
        "avoid_default":    "left",
        "avoid_turn_steps": _TURN_STEPS["robot2"],
    },
    "robot3": {
        "init_dir":         "right",
        "init_steps":       _steps_for_angle(2.2,  _INIT_OMEGA),  # ≈ 1 step
        "avoid_default":    "right",
        "avoid_turn_steps": _TURN_STEPS["robot3"],
    },
}
```

| Field | Purpose |
|-------|---------|
| `init_dir` | Direction to spin during startup heading correction (depends on spawn orientation in the `.wbt`) |
| `init_steps` | How many ticks to spin at startup — translates to the small angle each robot needs to face east |
| `avoid_default` | Fallback turn direction when an obstacle is dead-on (`robot1` and `robot2` have **opposite** defaults — head-on collisions between them resolve to opposite sides) |
| `avoid_turn_steps` | How long to spin during avoidance — **the symmetry-breaking parameter** |

### State labels

```python
INIT_TURN = "INIT_TURN"
DRIVE     = "DRIVE"
REVERSE   = "REVERSE"
TURN      = "TURN"
REORIENT  = "REORIENT"
```

Five named constants — typo-proof versions of the strings that represent each state. Every line of the main loop will reference them.

```
INIT_TURN  → spin once at startup to face east
   ↓
DRIVE      → cruise forward (default state, where most time is spent)
   ↓ (obstacle or beacon detected)
REVERSE    → back up
   ↓
TURN       → big dodge (90–120°)
   ↓
REORIENT   → small counter-spin (~45°) back toward east
   ↓
DRIVE      → resume cruising
```

`DRIVE` is the only state that lasts indefinitely; the others run for a fixed number of ticks then transition.

---

## Section 5 — Robot Initialization

Lines 104–136. Where the script wakes up its body — finds the motors, IR sensors, and receiver, and gets them ready to use.

### Background

**Velocity-controlled motors.** Webots motors have two modes:
1. **Position mode** — "go to angle X and stop" (like a servo)
2. **Velocity mode** — "spin continuously at X rad/s"

Setting position to **infinity** is Webots' convention for *"there is no target position, just spin at the velocity I give you."*

**Sample period for sensors.** Sensors don't read continuously — they sample. `enable(TIME_STEP)` tells the sensor "give me a fresh reading every 32 ms." Forget this and the sensor returns `NaN`.

**E-puck IR sensor layout** (looking down, robot's nose pointing right):

```
        ps7   ps0
       ↖       ↗
   ps6 ←  e-puck  → ps1
                  
       ps5         ps2
        ↖         ↗
          ps4  ps3
          (rear)
```

- **ps0, ps1** — right front
- **ps2, ps3** — right side / rear
- **ps4, ps5** — left rear
- **ps6, ps7** — left front

The code later reads `v[0], v[1], v[6], v[7]` — that's the **four front sensors**.

### The code

```python
robot = Robot()
name  = robot.getName()
cfg   = ROBOT_CONFIG.get(name, ROBOT_CONFIG["robot3"])

print(f"[{name}] online | init {cfg['init_dir']} {cfg['init_steps']} steps | "
      f"avoid default {cfg['avoid_default']} {cfg['avoid_turn_steps']} steps")

# Motors — velocity-controlled
lm = robot.getDevice("left wheel motor")
rm = robot.getDevice("right wheel motor")
for m in (lm, rm):
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

# IR proximity sensors
ps = []
for i in range(8):
    s = robot.getDevice(f"ps{i}")
    s.enable(TIME_STEP)
    ps.append(s)

# Receiver — listens on channel 1
receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)
receiver.setChannel(1)
```

### Key points

- **`ROBOT_CONFIG.get(name, ROBOT_CONFIG["robot3"])`** — defensive: if a robot's name isn't in the config, fall back to robot3's profile rather than crashing.
- **`m.setPosition(float("inf"))`** — puts the motor in velocity mode.
- **`receiver.setChannel(1)`** — must match the beacon's channel, or the signal is never received.

---

## Section 6 — Motion & Sensor Helpers

Lines 138–178. Building blocks the main loop will call repeatedly.

### Differential drive cheat sheet

| Left wheel | Right wheel | Result |
|------------|-------------|--------|
| forward    | forward     | drive straight forward |
| backward   | backward    | drive straight backward |
| backward   | forward     | spin in place left (CCW) |
| forward    | backward    | spin in place right (CW) |

### Motion helpers

```python
def wheels(left, right):
    lm.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    rm.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))

def spin_left(spd):  wheels(-spd,  spd)   # CCW
def spin_right(spd): wheels( spd, -spd)   # CW
```

The `max(-MAX_SPEED, min(MAX_SPEED, x))` is the classic **clamping trick**: clip values to safe limits.

### Sensor helpers

```python
def read_ps():
    return [s.getValue() for s in ps]

def front_blocked(v):
    return max(v[0], v[1], v[6], v[7]) > OBSTACLE_TH

def all_clear(v):
    return all(x < CLEAR_TH for x in v)   # defined but never used

def choose_avoid_dir(v, default):
    right_front = max(v[0], v[1])
    left_front  = max(v[6], v[7])
    if right_front > left_front + 20:
        return "left"
    if left_front > right_front + 20:
        return "right"
    return default
```

### `choose_avoid_dir` — smart dodge logic

| Sensor pattern | Decision |
|----------------|----------|
| Right side more blocked (by ≥ 20) | Turn **left** |
| Left side more blocked (by ≥ 20) | Turn **right** |
| Roughly equal (within 20) | Use per-robot **default** |

The **+20 buffer** is hysteresis again — don't act on noise. Only commit to a direction when one side is clearly more blocked.

---

## Section 7 — `classify_obstacle`

Lines 180–229. The biggest function in the file. **It does not change behavior** — it only prints flavorful Mars-themed labels in the log.

### Three signals it considers

| Signal | What it tells you |
|--------|-------------------|
| **peak** | how *close/big* the obstacle is |
| **symmetry** | how *square-on* the hit is |
| **side_max** | whether the obstacle wraps around (corner / cluster) |

```python
def classify_obstacle(v):
    rf       = max(v[0], v[1])
    lf       = max(v[6], v[7])
    peak     = max(rf, lf)
    symmetry = abs(rf - lf)
    side_max = max(v[2], v[3], v[4], v[5])
    ...
```

### Decision tree

```
peak > 2500   →   "ARENA BOUNDARY WALL (CORNER)"     if symmetry < 400 and side_max > 80
                  "ARENA BOUNDARY WALL"               if symmetry < 400
                  "LARGE BASALT BOULDER"              else

peak > 800    →   "VOLCANIC ROCK FORMATION"           if symmetry < 300
                  "BASALT ROCK (GLANCING HIT)"        else

peak > 200    →   "LOOSE ROCK CLUSTER"                if side_max > 120
                  "SMALL ROCK FRAGMENT"               else

peak ≤ 200    →   "METALLIC DEBRIS FIELD"
```

### Worked example

Robot drives straight into a wall with `v = [3500, 3400, 60, 30, 20, 25, 3300, 3500]`:
- `peak = 3500` → branch 1
- `symmetry = 0` → `< 400`
- `side_max = 60` → `< 80`
- → **"ARENA BOUNDARY WALL"** ✓

### Real purpose

Pure **flavor / debugging**. The robot dodges identically regardless of what label it printed. If you ever change the world (new rocks, different sizes), this is the function whose thresholds you'd retune.

---

## Section 8 — State Variables & Main Loop Top

Lines 231–263.

### State machine variables

```python
state          = INIT_TURN
phase_step     = 0       # ticks elapsed in current state
drive_steps    = 0       # cumulative DRIVE ticks (never read — dead code)
avoid_dir      = cfg["avoid_default"]
obstacle_count = 0       # cumulative collisions, for log decoration

beacon_found   = False   # True once "BEACON" has been heard
last_nasa_time = 0.0     # sim-time of last NASA uplink
```

| Variable | Role |
|----------|------|
| `state` | Current mode (`INIT_TURN` / `DRIVE` / `REVERSE` / `TURN` / `REORIENT`) |
| `phase_step` | Ticks spent in the *current* state. Reset to 0 every state change |
| `drive_steps` | Cumulative drive ticks. Defined and incremented but **never read** |
| `avoid_dir` | Current dodge direction. Set by `choose_avoid_dir` when an event fires |
| `obstacle_count` | Total collisions ever, for log tagging |
| `beacon_found` | Persistent flag — unlocks the NASA uplink prints |
| `last_nasa_time` | Used with `NASA_INTERVAL_S = 600` to gate periodic uplinks |

### Two beacon flags — why

| Flag | Scope | Meaning |
|------|-------|---------|
| `beacon_signal_active` | per-tick (local) | *Heard the beacon RIGHT NOW?* |
| `beacon_found` | persistent | *Have I EVER heard the beacon?* |

The robot logs the **first** find using `beacon_found` (one-time celebration). Every subsequent re-detection logs `RE-ESTABLISHING UPLINK` using `beacon_signal_active`.

### Main loop top

```python
while robot.step(TIME_STEP) != -1:

    v        = read_ps()
    sim_time = robot.getTime()

    # Drain receiver queue — check for beacon signal
    beacon_signal_active = False
    while receiver.getQueueLength() > 0:
        try:
            msg = receiver.getString()
        except Exception:
            msg = receiver.getData().decode("utf-8")
        if msg == BEACON_SIGNAL:
            beacon_signal_active = True
        receiver.nextPacket()
```

**Receiver queue.** When the beacon broadcasts, messages stack up in a queue. Each tick we **drain** the queue: read every pending message, call `nextPacket()` to discard it. Without draining, the queue grows forever.

**`try / except`** — defensive cross-version compatibility. `getString()` works on newer Webots; `getData().decode()` is the fallback for older versions.

---

## Section 9 — NASA Uplink & `INIT_TURN`

Lines 264–287.

### NASA periodic uplink

```python
if beacon_found and (sim_time - last_nasa_time) >= NASA_INTERVAL_S:
    mins = int(sim_time // 60)
    secs = int(sim_time % 60)
    print(f"[{name}] ============================================================")
    print(f"[{name}] SENDING DATA TO NASA - T+{mins}m{secs:02d}s MISSION ELAPSED")
    print(f"[{name}] MARS SURFACE TELEMETRY UPLINK ACTIVE")
    print(f"[{name}] BEACON COORDINATES TRANSMITTED - AWAITING ACKNOWLEDGEMENT")
    print(f"[{name}] ============================================================")
    last_nasa_time = sim_time
```

- Sits **outside** the state-machine ladder — fires on its own timer regardless of what the robot is doing.
- Two conditions, both must be true: `beacon_found` AND `≥ 600 sim-seconds since last uplink`.
- The 10-minute clock starts when the beacon is **first found**, not when the simulation starts.
- `mins / secs` formatting uses integer division (`//`) and modulo (`%`) to convert seconds into `MMmSSs`.
- **No behavioral effect** — only prints and updates `last_nasa_time`.

### `INIT_TURN` state — startup heading correction

```python
if state == INIT_TURN:
    if phase_step < cfg["init_steps"]:
        if cfg["init_dir"] == "left":
            spin_left(INIT_TURN_SPEED)
        else:
            spin_right(INIT_TURN_SPEED)
        phase_step += 1
    else:
        print(f"[{name}] heading locked — entering DRIVE")
        state, phase_step = DRIVE, 0
```

The robot's first behavior. Spins slowly (`INIT_TURN_SPEED = 2.0`) for a per-robot number of ticks, pointing it roughly east, then transitions to `DRIVE`.

### The "timed-state pattern" used by every state except DRIVE

```
if phase_step < TARGET:
    do_motion()
    phase_step += 1
else:
    state, phase_step = NEXT, 0
```

You'll see this exact shape in `INIT_TURN`, `REVERSE`, `TURN`, and `REORIENT`. Only `DRIVE` is different — it has no time limit, it ends on a sensor event.

### Subtle Python detail

When a state's `else` branch fires `state = DRIVE`, the **next** `elif state == DRIVE:` is **not** triggered on the same tick. Once any branch of an `if/elif` runs, the rest are skipped. So state transitions take effect on the **next** tick (32 ms later — invisible).

---

## Section 10 — The `DRIVE` State

Lines 289–337. The **heart** of the controller. The robot spends ~99% of its life here.

### Three branches, evaluated in priority order

```python
elif state == DRIVE:
    if beacon_signal_active:        # Priority 1
        ...
    elif front_blocked(v):          # Priority 2
        ...
    else:                            # Default: cruise
        wheels(DRIVE_SPEED, DRIVE_SPEED)
        drive_steps += 1
```

**Why beacon comes first:** the beacon sphere is *also* a physical object — its IR returns will trigger `front_blocked(v)`. But because the beacon branch is first, the radio signal wins. Without that ordering, the robot would mistake the beacon for a small rock and never log a find.

### Branch 1 — Beacon signal received

```python
if beacon_signal_active:
    if not beacon_found:
        sep = "=" * 60
        print(f"[{name}] {sep}")
        print(f"[{name}] BEACON FOUND - RADIO SIGNAL CONFIRMED")
        print(f"[{name}] BEACON IDENTIFIED: SPHERE r=0.1m COLOR 1 0.85 0")
        print(f"[{name}] SENDING DATA TO NASA")
        print(f"[{name}] MARS BEACON SECURED - TRANSMITTING COORDINATES")
        print(f"[{name}] UPLINK ACTIVE - MISSION CONTROL NOTIFIED")
        print(f"[{name}] {sep}")
        beacon_found   = True
        last_nasa_time = sim_time
    else:
        peak = int(max(v[0], v[1], v[6], v[7]))
        print(f"[{name}] BEACON SIGNAL ACTIVE - [SENSOR PEAK: {peak}] - RE-ESTABLISHING UPLINK")
    avoid_dir = choose_avoid_dir(v, cfg["avoid_default"])
    state, phase_step = REVERSE, 0
```

- **First time** (`not beacon_found`): print the dramatic celebration block, set `beacon_found = True`, start the NASA uplink clock with `last_nasa_time = sim_time`.
- **Subsequent times**: brief one-line `RE-ESTABLISHING UPLINK` log.
- **Either way**: pick a dodge direction and transition to `REVERSE`. The robot **backs off from the beacon** so it can keep exploring instead of camping on it.

### Branch 2 — Regular obstacle

```python
elif front_blocked(v):
    peak      = int(max(v[0], v[1], v[6], v[7]))
    avoid_dir = choose_avoid_dir(v, cfg["avoid_default"])
    obj_type  = classify_obstacle(v)
    obstacle_count += 1
    hit_msgs = { ... }   # 8 dramatic messages keyed off obj_type
    hit_msg = hit_msgs.get(obj_type, f"OBSTACLE HIT - {obj_type} DETECTED")
    print(f"[{name}] {hit_msg} [SENSOR PEAK: {peak}] - OBSTACLE #{obstacle_count} - INITIATING AVOIDANCE")
    state, phase_step = REVERSE, 0
```

- Pick dodge direction.
- Classify obstacle (flavor only).
- Translate canonical label → dramatic Mars-flavored message.
- Print, transition to `REVERSE`.

### Branch 3 — Cruise

```python
else:
    wheels(DRIVE_SPEED, DRIVE_SPEED)
    drive_steps += 1
```

Both wheels at 5.5 rad/s → straight-forward motion. Stay in DRIVE until something happens.

### Behavioral flow

```
                  ┌─────────────┐
                  │   DRIVE     │
                  │ (this tick) │
                  └──────┬──────┘
                         │
                  ┌──────┴──────┐
                  │ heard       │
                  │ "BEACON"?   │
                  └──┬───────┬──┘
                yes  │       │  no
       ┌─────────────▼┐   ┌──▼────────────────┐
       │ Log find /   │   │ front IR > 80?    │
       │ re-find      │   └──┬────────────┬───┘
       │ → REVERSE    │  yes │            │ no
       └──────────────┘      │            │
                  ┌──────────▼──┐   ┌────▼─────┐
                  │ classify,   │   │ wheels   │
                  │ log hit,    │   │ forward  │
                  │ → REVERSE   │   │ stay in  │
                  └─────────────┘   │  DRIVE   │
                                    └──────────┘
```

---

## Section 11 — `REVERSE`, `TURN`, `REORIENT`

Lines 339–375. The avoidance routine. Three timed states played back-to-back.

### Why the routine is "fire-and-forget"

DRIVE made the smart decisions (`avoid_dir`, `avoid_turn_steps`). The three following states **execute the plan blindly** — no sensor checks, no second-guessing. Even if the obstacle disappears partway through, the robot completes the full routine.

### `REVERSE` — back away from the obstacle

```python
elif state == REVERSE:
    if phase_step < REVERSE_STEPS:
        wheels(-DRIVE_SPEED * 0.55, -DRIVE_SPEED * 0.55)
        phase_step += 1
    else:
        print(f"[{name}] reversed — entering TURN ({avoid_dir})")
        state, phase_step = TURN, 0
```

- 22 ticks at -3.025 rad/s (≈ 55% of drive speed).
- Backing up slowly is safer (rear sensors aren't checked).
- Gives clearance for the upcoming spin.

### `TURN` — big rotation away from the obstacle

```python
elif state == TURN:
    if phase_step < cfg["avoid_turn_steps"]:
        if avoid_dir == "left":
            spin_left(TURN_SPEED)
        else:
            spin_right(TURN_SPEED)
        phase_step += 1
    else:
        print(f"[{name}] turn done — entering REORIENT")
        state, phase_step = REORIENT, 0
```

- Per-robot duration: robot1 = 21 ticks (90°), robot2 = 24 (105°), robot3 = 28 (120°).
- Uses the **fast** `TURN_SPEED = 3.0` rad/s — efficient for big angles.
- Direction comes from DRIVE's `choose_avoid_dir` decision.

### `REORIENT` — partial counter-spin back toward east

```python
elif state == REORIENT:
    if phase_step < REORIENT_STEPS:
        if avoid_dir == "left":
            spin_right(INIT_TURN_SPEED)
        else:
            spin_left(INIT_TURN_SPEED)
        phase_step += 1
    else:
        print(f"[{name}] reoriented — back to DRIVE")
        state, phase_step = DRIVE, 0
```

- 16 ticks at the slow `INIT_TURN_SPEED = 2.0` rad/s (≈ 45°).
- Direction is **opposite** to `avoid_dir` — partial undo of the big TURN.
- Slow speed because 45° is small and we want precision.
- Doesn't fully undo TURN — leaves a deliberate **net rotation** so the robot doesn't head back at what it just dodged.

### Net heading change after a full dodge

| Robot  | TURN  | REORIENT | Net rotation |
|--------|-------|----------|--------------|
| robot1 | 90°   | 45°      | **45°**      |
| robot2 | 105°  | 45°      | **60°**      |
| robot3 | 120°  | 45°      | **75°**      |

So no robot drifts straight east anymore after a collision — each veers off at a different angle. Over many collisions, this causes each robot to wander a unique path through the arena.

### Full dodge timeline (robot1, 90° dodge to the right)

| Tick   | State    | Action                                | phase_step |
|--------|----------|---------------------------------------|------------|
| 0      | DRIVE    | Detect obstacle, pick `avoid_dir`     | →REVERSE,0 |
| 1–22   | REVERSE  | Back up at -3.025 rad/s               | 0 → 22     |
| 23     | REVERSE  | Print, transition                     | →TURN, 0   |
| 24–44  | TURN     | Spin right at 3.0 rad/s               | 0 → 21     |
| 45     | TURN     | Print, transition                     | →REORIENT,0|
| 46–61  | REORIENT | Spin **left** at 2.0 rad/s            | 0 → 16     |
| 62     | REORIENT | Print, transition                     | →DRIVE, 0  |
| 63+    | DRIVE    | Cruise resumes (or new event)         | n/a        |

Total: ~62 ticks ≈ 2 seconds per collision. Then back to driving.

---

## Whole-System Summary

### Recap of the two files

**`beacon_controller.py`** (5 lines of real code):
- Non-moving Robot
- Broadcasts `"BEACON"` on channel 1, range 0.5 m, every 32 ms
- Pure proximity beacon — no direction information

**`astroswarm_controller.py`** (~370 lines):
- **Setup:** constants, per-robot config, motor/sensor/receiver init, motion/sensor helpers, obstacle classifier
- **Main loop:** every tick → snapshot sensors and time → drain receiver to check for beacon → fire NASA uplink if it's time → run state machine
- **State machine:** five states
  - `INIT_TURN` faces east at startup
  - `DRIVE` cruises and makes decisions
  - `REVERSE / TURN / REORIENT` execute the dodge routine
- **Symmetry breaking:** different init angles, different avoid angles, opposite avoid defaults — three robots with the same code but distinct behavior
- **Mars flavor:** `classify_obstacle` and the various print blocks turn dry sensor data into mission-control storytelling

### Key engineering ideas demonstrated

| Idea                       | Where it appears                                    |
|----------------------------|-----------------------------------------------------|
| State machine              | The five states + transitions                       |
| Hysteresis (two thresholds)| `OBSTACLE_TH = 80` / `CLEAR_TH = 40`                |
| Symmetry breaking          | Per-robot `avoid_turn_steps` and `avoid_default`    |
| Differential drive         | `wheels(left, right)` and the spin helpers          |
| Velocity-mode motors       | `setPosition(float("inf"))`                         |
| Sensor sampling            | `enable(TIME_STEP)`                                 |
| Defensive lookups          | `ROBOT_CONFIG.get(...)`, `hit_msgs.get(...)`        |
| Cross-version compatibility| `try / except` around `receiver.getString()`        |
| Time-based events          | NASA uplink interval check                          |
| Heuristic classification   | `classify_obstacle` — peak / symmetry / side_max    |
| Fire-and-forget pipelines  | REVERSE → TURN → REORIENT (no mid-routine checks)   |

### What the swarm achieves

- Three independent agents
- Identical code, different personalities (via per-robot config)
- Each one is a dumb wanderer alone, but the trio covers more ground than one robot would
- Detection is binary and short-range, so the simulation rewards spatial coverage — exactly what the swarm provides
- Once any robot finds the beacon, the mission is logged as a success, and the robot keeps exploring (rather than stopping) so the swarm continues providing telemetry
