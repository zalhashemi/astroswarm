"""
AstroSwarm Controller
Yousif Abuzuhaira & Zain AlHashemi
"""

from controller import Robot
import math

# ─────────────────────────────────────────────────────────────────────────────
# Simulation & hardware constants
# ─────────────────────────────────────────────────────────────────────────────
TIME_STEP        = 32           # ms per Webots step
DT               = TIME_STEP / 1000.0   # 0.032 s
MAX_SPEED        = 6.28         # rad/s — e-puck motor limit

DRIVE_SPEED      = 5.5          # cruise forward speed (rad/s per wheel)
TURN_SPEED       = 3.0          # obstacle-avoidance spin speed
INIT_TURN_SPEED  = 2.0          # initial heading-correction spin speed (slower = more precise)

# E-puck kinematics
WHEEL_RADIUS  = 0.0205          # m
AXLE_LENGTH   = 0.052           # m, centre-to-centre of wheels

# Angular velocity (rad/s) when spinning in-place at a given wheel speed w:
#   ω_body = 2 * w * WHEEL_RADIUS / AXLE_LENGTH
_INIT_OMEGA = 2.0 * INIT_TURN_SPEED * WHEEL_RADIUS / AXLE_LENGTH  # ≈ 1.577 rad/s
_TURN_OMEGA = 2.0 * TURN_SPEED      * WHEEL_RADIUS / AXLE_LENGTH  # ≈ 2.365 rad/s

def _steps_for_angle(degrees, omega):
    """Return the number of TIME_STEP ticks to rotate `degrees` at `omega` rad/s."""
    return max(1, int(round(math.radians(abs(degrees)) / (omega * DT))))

# ─────────────────────────────────────────────────────────────────────────────
# Sensor thresholds
# ─────────────────────────────────────────────────────────────────────────────
OBSTACLE_TH = 80    # value that triggers obstacle response
CLEAR_TH    = 40    # value considered "open air"

# Beacon detection via Receiver sensor.
#
# The beacon Robot node runs beacon_controller.py which broadcasts "BEACON"
# on Emitter channel 1 with range 0.5 m every step. 
# When a robot's receiver picks up the "BEACON" signal it is within 0.5 m of the beacon 
#
BEACON_SIGNAL    = "BEACON"   # message broadcast by beacon_controller

# NASA uplink interval in simulation seconds (10 sim-minutes)
NASA_INTERVAL_S  = 600.0

# ─────────────────────────────────────────────────────────────────────────────
# Avoidance timing
# ─────────────────────────────────────────────────────────────────────────────
REVERSE_STEPS  = 22     # steps to back up before turning

# Each robot turns a slightly different amount so they don't oscillate in sync.
# After the avoidance turn, counter-spin at INIT_TURN_SPEED to partially
# restore the beacon heading.  ~45° of counter-turn keeps the robot
# aimed somewhat eastward without overshooting.
REORIENT_STEPS = _steps_for_angle(45, _INIT_OMEGA)  # ≈ 16 steps

# Steps each robot spins during an avoidance turn — deliberately different so
# robots don't re-synchronise and oscillate in-phase after collisions.
_TURN_STEPS = {
    "robot1": _steps_for_angle( 90, _TURN_OMEGA),   # ≈ 21 steps
    "robot2": _steps_for_angle(105, _TURN_OMEGA),   # ≈ 24 steps
    "robot3": _steps_for_angle(120, _TURN_OMEGA),   # ≈ 28 steps
}

# ─────────────────────────────────────────────────────────────────────────────
# Per-robot configuration
# ─────────────────────────────────────────────────────────────────────────────
# "avoid_default" is used when both front quadrants are equally blocked.
# Robots are assigned opposite defaults to break symmetry.
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

# ─────────────────────────────────────────────────────────────────────────────
# State labels
# ─────────────────────────────────────────────────────────────────────────────
INIT_TURN = "INIT_TURN"
DRIVE     = "DRIVE"
REVERSE   = "REVERSE"
TURN      = "TURN"
REORIENT  = "REORIENT"

# ─────────────────────────────────────────────────────────────────────────────
# Initialise robot
# ─────────────────────────────────────────────────────────────────────────────
robot = Robot()
name  = robot.getName()
cfg   = ROBOT_CONFIG.get(name, ROBOT_CONFIG["robot3"])

print(
    f"[{name}] online | "
    f"init {cfg['init_dir']} {cfg['init_steps']} steps | "
    f"avoid default {cfg['avoid_default']} {cfg['avoid_turn_steps']} steps"
)

# Motors — velocity-controlled (position set to infinity)
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

# Receiver — listens for the beacon's "BEACON" broadcast on channel 1.
# The beacon Robot node runs beacon_controller.py which emits on channel 1
# with range 0.5 m.  Signal received → robot is within 0.5 m of the beacon.
receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)
receiver.setChannel(1)   # tune to beacon broadcast channel

# ─────────────────────────────────────────────────────────────────────────────
# Motion helpers
# ─────────────────────────────────────────────────────────────────────────────
def wheels(left, right):
    """Set wheel velocities, clamped to hardware limits."""
    lm.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    rm.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))

def spin_left(spd):
    wheels(-spd,  spd)   # counter-clockwise (left turn)

def spin_right(spd):
    wheels( spd, -spd)   # clockwise (right turn)

# ─────────────────────────────────────────────────────────────────────────────
# Sensor helpers
# ─────────────────────────────────────────────────────────────────────────────
def read_ps():
    return [s.getValue() for s in ps]

def front_blocked(v):
    """True if any forward-facing sensor exceeds the obstacle threshold."""
    return max(v[0], v[1], v[6], v[7]) > OBSTACLE_TH

def all_clear(v):
    """True if every sensor reads open air."""
    return all(x < CLEAR_TH for x in v)

def choose_avoid_dir(v, default):
    """
    Pick a turn direction that opens the most space in front.
    If the right-front quadrant is more blocked, turn left (and vice-versa).
    Fall back to the robot's default when the two quadrants are similar.
    """
    right_front = max(v[0], v[1])
    left_front  = max(v[6], v[7])
    if right_front > left_front + 20:
        return "left"    # obstacle on right front → swing left to clear
    if left_front > right_front + 20:
        return "right"   # obstacle on left  front → swing right to clear
    return default       # symmetric or ambiguous → use robot's default

def classify_obstacle(v):
    """
    Infer what the robot hit from the IR sensor signature.
    Does NOT handle beacon detection — that is done in the DRIVE state
    using the zone guard (drive_steps) combined with this signature data.

    IR intensity heuristics (Webots e-puck, 4x4 m arena):
      peak > 2500  flat, intense return  → arena boundary wall
      peak > 800   moderate              → large volcanic rock (box obstacle)
      peak > 200   low, concentrated     → smaller loose rock
      peak ≤ 200   barely detectable     → metallic debris or far-off edge

    Symmetry: |right_front − left_front|
      low  → flat surface directly ahead (wall or face-on rock)
      high → angled hit on one corner of an obstacle (rock edge)

    Side-sensor co-activation hints at a corner trap (wall corner).
    """
    rf       = max(v[0], v[1])   # right-front quadrant
    lf       = max(v[6], v[7])   # left-front  quadrant
    peak     = max(rf, lf)
    symmetry = abs(rf - lf)      # low = dead-on, high = glancing/angled
    side_max = max(v[2], v[3], v[4], v[5])

    if peak > 2500:
        # Very strong return — flat hard surface up close
        if symmetry < 400 and side_max > OBSTACLE_TH:
            return "ARENA BOUNDARY WALL (CORNER)"
        elif symmetry < 400:
            return "ARENA BOUNDARY WALL"
        else:
            return "LARGE BASALT BOULDER"

    elif peak > 800:
        # Strong but not maxed — sizeable box rock
        if symmetry < 300:
            return "VOLCANIC ROCK FORMATION"
        else:
            return "BASALT ROCK (GLANCING HIT)"

    elif peak > 200:
        # Moderate — smaller rock, possibly another rover
        if side_max > OBSTACLE_TH * 1.5:
            return "LOOSE ROCK CLUSTER"
        else:
            return "SMALL ROCK FRAGMENT"

    else:
        # Barely over threshold — tiny object or far edge
        return "METALLIC DEBRIS FIELD"

# ─────────────────────────────────────────────────────────────────────────────
# State machine variables
# ─────────────────────────────────────────────────────────────────────────────
state          = INIT_TURN
phase_step     = 0     # steps elapsed in the current state
drive_steps    = 0     # cumulative steps spent in forward DRIVE (distance proxy)
avoid_dir      = cfg["avoid_default"]   # direction chosen for current avoidance
obstacle_count = 0     # total obstacles encountered (for log tagging)

beacon_found   = False   # True once the beacon sphere has been sensed
last_nasa_time = 0.0     # sim-time of the last NASA uplink (seconds)

# ─────────────────────────────────────────────────────────────────────────────
# Main control loop
# ─────────────────────────────────────────────────────────────────────────────
while robot.step(TIME_STEP) != -1:

    v        = read_ps()
    sim_time = robot.getTime()   # simulation time in seconds

    # ── Drain receiver queue; check for beacon signal ─────────────────────────
    # The beacon broadcasts "BEACON" on channel 1 with range 0.5 m every step.
    # If our receiver has any packets, we are within 0.5 m of the beacon.
    beacon_signal_active = False
    while receiver.getQueueLength() > 0:
        try:
            msg = receiver.getString()
        except Exception:
            msg = receiver.getData().decode("utf-8")
        if msg == BEACON_SIGNAL:
            beacon_signal_active = True
        receiver.nextPacket()

    # ── NASA periodic uplink ──────────────────────────────────────────────────
    # After beacon has been found, broadcast data every NASA_INTERVAL_S sim-seconds.
    if beacon_found and (sim_time - last_nasa_time) >= NASA_INTERVAL_S:
        mins = int(sim_time // 60)
        secs = int(sim_time % 60)
        print(f"[{name}] ============================================================")
        print(f"[{name}] SENDING DATA TO NASA - T+{mins}m{secs:02d}s MISSION ELAPSED")
        print(f"[{name}] MARS SURFACE TELEMETRY UPLINK ACTIVE")
        print(f"[{name}] BEACON COORDINATES TRANSMITTED - AWAITING ACKNOWLEDGEMENT")
        print(f"[{name}] ============================================================")
        last_nasa_time = sim_time

    # ── INIT_TURN ─────────────────────────────────────────────────────────────
    # Rotate once at startup to face the beacon, then enter DRIVE.
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

    # ── DRIVE ─────────────────────────────────────────────────────────────────
    # Cruise straight east.  Exit condition: obstacle / beacon ahead → REVERSE.
    # Robots explore indefinitely.
    elif state == DRIVE:
        # ── Beacon signal received ────────────────────────────────────────────
        # The receiver picked up the beacon's radio broadcast (range 0.5 m).
        # We are within half a metre of the sphere — confirm as beacon contact.
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
            # Back off from the beacon and keep exploring
            avoid_dir = choose_avoid_dir(v, cfg["avoid_default"])
            state, phase_step = REVERSE, 0

        elif front_blocked(v):
            # ── Regular obstacle ──────────────────────────────────────────────
            peak      = int(max(v[0], v[1], v[6], v[7]))
            avoid_dir = choose_avoid_dir(v, cfg["avoid_default"])
            obj_type  = classify_obstacle(v)
            obstacle_count += 1
            hit_msgs = {
                "ARENA BOUNDARY WALL (CORNER)": "WALL CORNER HIT - BOUNDARY DETECTED",
                "ARENA BOUNDARY WALL":          "WALL HIT - ARENA BOUNDARY DETECTED",
                "LARGE BASALT BOULDER":         "ROCK HIT - LARGE BASALT BOULDER DETECTED",
                "VOLCANIC ROCK FORMATION":      "ROCK HIT - VOLCANIC ROCK FORMATION DETECTED",
                "BASALT ROCK (GLANCING HIT)":   "ROCK HIT - BASALT ROCK GLANCING BLOW",
                "LOOSE ROCK CLUSTER":           "ROCK HIT - LOOSE ROCK CLUSTER DETECTED",
                "SMALL ROCK FRAGMENT":          "ROCK HIT - SMALL ROCK FRAGMENT DETECTED",
                "METALLIC DEBRIS FIELD":        "DEBRIS HIT - METALLIC DEBRIS FIELD DETECTED",
            }
            hit_msg = hit_msgs.get(obj_type, f"OBSTACLE HIT - {obj_type} DETECTED")
            print(f"[{name}] {hit_msg} [SENSOR PEAK: {peak}] - OBSTACLE #{obstacle_count} - INITIATING AVOIDANCE")
            state, phase_step = REVERSE, 0
        else:
            # Clear ahead — drive straight.
            wheels(DRIVE_SPEED, DRIVE_SPEED)
            drive_steps += 1

    # ── REVERSE ───────────────────────────────────────────────────────────────
    # Back up before turning so the robot is never spinning against a wall.
    elif state == REVERSE:
        if phase_step < REVERSE_STEPS:
            wheels(-DRIVE_SPEED * 0.55, -DRIVE_SPEED * 0.55)
            phase_step += 1
        else:
            print(f"[{name}] reversed — entering TURN ({avoid_dir})")
            state, phase_step = TURN, 0

    # ── TURN ──────────────────────────────────────────────────────────────────
    # Rotate away from the obstacle for a fixed number of steps.
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

    # ── REORIENT ──────────────────────────────────────────────────────────────
    # Partially counter-spin to recover a roughly eastward heading.
    # We turn back opposite to avoid_dir for REORIENT_STEPS at the slower
    # INIT_TURN_SPEED, giving a net ~45° correction back toward east.
    elif state == REORIENT:
        if phase_step < REORIENT_STEPS:
            if avoid_dir == "left":
                spin_right(INIT_TURN_SPEED)   # avoidance was left → nudge right
            else:
                spin_left(INIT_TURN_SPEED)    # avoidance was right → nudge left
            phase_step += 1
        else:
            print(f"[{name}] reoriented — back to DRIVE")
            state, phase_step = DRIVE, 0
