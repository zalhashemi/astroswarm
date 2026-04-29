# AstroSwarm Presentation Script
### Two-Presenter Format — Yousif & Zain

**Color key:** **[YOUSIF]** speaks first, **[ZAIN]** alternates. Italic notes in brackets are stage directions, not lines to read.

---

## SLIDE 1 — Title: "Astroswarm: Design and Implementation of Space Robotic Rovers in Simulation"

**[YOUSIF]**
Good morning everyone. Our project is called *AstroSwarm* — the design and implementation of space robotic rovers in simulation. I'm Yousif Abuzuhaira…

**[ZAIN]**
…and I'm Zain Alhashemi. Over the next few minutes we'll walk you through why we built this, how it works, and we'll show you a live demo of three rovers searching for a beacon on a simulated Mars surface.

---

## SLIDE 2 — The Problem

**[YOUSIF]**
So let's start with the problem we're trying to address. NASA currently sends individual rovers to Mars that cost over 2.5 billion US dollars each. The issue with that model is straightforward — if that single rover fails, the entire mission is lost. That's a huge risk to put on one piece of hardware, and over time it becomes an unsustainable use of resources and money to keep building bigger and more expensive single rovers.

**[ZAIN]**
Our project explores an alternative — *swarm robotics*. The idea is to deploy multiple smaller, cheaper robots that work together to cover more terrain at the same time. If one fails, the mission continues. Now, we obviously can't ship hardware to Mars to test this, so we built a simulation. The simulation gives us a safe, low-cost way to develop and validate swarm algorithms before any real-world deployment.

---

## SLIDE 3 — Objectives (Design / Implementation / Evaluation)

**[ZAIN]**
We split our objectives into three categories. On the *design* side, we needed to build a virtual Mars surface in Webots — terrain, rocks, boundary walls, and a target beacon — and we needed to model three e-puck robots with the right sensors and actuators.

**[YOUSIF]**
On the *implementation* side, we wrote an intelligent state machine in Python that controls each robot, with algorithms that detect obstacles using the IR sensors, detect the beacon using a radio receiver, and we gave each robot a unique starting heading and a unique avoidance turn so they fan out across the arena instead of clumping together.

**[ZAIN]**
And on the *evaluation* side, we wanted to measure how well each robot reaches the beacon from its starting position without permanently failing, and log every obstacle and beacon detection in the console for each robot independently throughout the simulation.

---

## SLIDE 4 — Tools and Technologies Used

**[YOUSIF]**
For the simulation platform we used Cyberbotics Webots R2023b along with the Webots PROTO system for our world objects. Everything is written in Python 3, using the Webots Python API — that gives us direct access to the robot's motors and sensors through classes like `Robot`, `Motor`, `DistanceSensor`, and `Receiver`.

**[ZAIN]**
The robot model itself is the built-in Webots e-puck. It's a small two-wheeled disc — wheel radius about 2 centimeters, axle length about 5 centimeters, max wheel speed of 6.28 radians per second, eight infrared proximity sensors arranged around the body, and a radio receiver tuned to channel 1 with a 0.5 meter detection range. The world was built in the Webots Scene Tree editor and saved as a `.wbt` file with a Mars background texture, and we did all our editing in Visual Studio Code.

---

## SLIDE 5 — Figure 1: Block Diagram

**[ZAIN]**
This block diagram shows the high-level system. On the left you've got the inputs — the IR sensors and the radio receiver feeding into the controller. The controller is a state machine that decides what the robot should do, and it sends velocity commands out to the two wheel motors. The beacon sphere is independent — it just broadcasts a signal that any robot in range can pick up. Three robots, one beacon, all running their own controller in parallel.

---

## SLIDE 6 — Design Methodology

**[YOUSIF]**
Our methodology had a few key pillars. The *overall approach* is a state machine design — every robot is always in exactly one mode at a time. The *initial heading correction* gives each robot a small, slightly different turn at startup so they don't all drive the same direction. The *obstacle detection and classification* reads the front IR sensors, decides if something's blocking the path, and labels the type of obstacle for the log.

**[ZAIN]**
The *fan-out path divergence* is the symmetry-breaking piece — each robot has different avoidance angles, so after a collision they don't end up moving in lockstep. *Beacon detection* uses the radio receiver, not the IR sensors, because the beacon is a real physical object that the IR would otherwise misclassify as just another rock. The *avoidance sequence* is a fixed three-step routine — reverse, turn, reorient — and finally the *NASA uplink scheduling* prints periodic mission telemetry once any robot finds the beacon.

---

## SLIDE 7 — Table 1: Component Descriptions

**[YOUSIF]**
This table summarizes the actual components. We have three e-puck robots — that's our autonomous swarm. Each robot has eight IR proximity sensors named ps0 through ps7 with values from 0 for clear to 4096 for direct contact, two velocity-controlled wheel motors, and one beacon receiver tuned to channel 1.

**[ZAIN]**
On the world side we have the beacon sphere itself — it's the yellow target sitting at coordinates (1.2, 0.0, 0.1), 10 centimeters in radius, broadcasting on channel 1 at half a meter range. There are five rock obstacles scattered around — basically brown box geometries between 20 and 30 centimeters with collision boxes. Four arena walls enclose a 4-by-4 meter space, and the Mars floor is a single 4-by-4 meter plane with the red-orange Mars texture.

---

## SLIDE 8 — Figure 2: Flowchart

**[YOUSIF]**
The flowchart here shows the controller logic from the inside. Every simulation tick — that's every 32 milliseconds — the robot reads its sensors, drains the receiver queue, then runs through the state machine. If it's in DRIVE and it hears the beacon, it celebrates and goes into the avoidance routine. If it's in DRIVE and the front IR is blocked, it classifies the obstacle and goes into avoidance. Otherwise it just keeps cruising forward. Avoidance is a fixed sequence — REVERSE, TURN, REORIENT — and then back to DRIVE.

---

## SLIDE 9 — LIVE DEMO

**[ZAIN]**
Alright — at this point we'd like to switch over to the live demo. We're going to launch the simulation in Webots, and as it runs we'll walk you through the actual code that's making everything happen. Yousif and I will take turns explaining different parts of the controller.

*[Switch to Webots window. Start the simulation. Then switch to the code editor.]*

---

# LIVE DEMO — CODE WALKTHROUGH SCRIPT

> Roughly alternating, eleven sections total. Yousif takes 6, Zain takes 5.

---

### CODE PART 1 — `beacon_controller.py` (Yousif)

**[YOUSIF]**
Let's start with the simpler of the two files — `beacon_controller.py`. This script runs *inside* the glowing yellow sphere you see in the arena. Even though the sphere doesn't move, in Webots anything that has a controller is technically a "Robot," so we instantiate one. We grab the emitter device, set our time step to 32 milliseconds, and then in the main loop we just broadcast the string `"BEACON"` on channel 1 every single tick. The range and channel themselves are configured on the emitter device in the world file, not in the Python code. The whole script is essentially five lines of real logic — every 32 milliseconds it shouts "BEACON" into the void, and any robot within half a meter hears it.

The important conceptual point is that the beacon is a *proximity confirmation tool*, not a GPS. It doesn't tell the robot which direction it is — only that the robot is close enough to hear it. Like a smoke alarm beeping in a dark house: you can hear the beep, but you can't tell which room it's in.

---

### CODE PART 2 — Imports & Hardware Constants (Zain)

**[ZAIN]**
Now we move to `astroswarm_controller.py`, which is the brain that runs inside each of the three e-pucks. The first chunk of the file is constants. The time step is 32 milliseconds, the max wheel speed is 6.28 radians per second — that's exactly one full rotation per second. We define three different motion speeds: `DRIVE_SPEED` of 5.5 for cruising, `TURN_SPEED` of 3.0 for the big avoidance spins, and `INIT_TURN_SPEED` of 2.0 for the slow precise startup turn. The rule of thumb is: big turn — fast speed, small turn — slow speed.

Below the constants we use the differential drive formula — `omega equals two times wheel speed times wheel radius divided by axle length` — to convert wheel speeds into body rotation rates. And then `_steps_for_angle` is a helper that figures out how many 32-millisecond ticks the robot needs to spin to achieve a target *angle*. So instead of using a gyroscope, we just count ticks. For example, a 90-degree turn at our `TURN_OMEGA` works out to roughly 21 ticks, which is about 670 milliseconds of spinning.

---

### CODE PART 3 — Sensor Thresholds & Avoidance Timing (Yousif)

**[YOUSIF]**
Next up are the sensor thresholds and the avoidance timing. The IR sensors return higher numbers the closer something is — zero means clear, hundreds or thousands mean something's right in front. We use *two* thresholds instead of one — `OBSTACLE_TH` at 80 to trigger an obstacle response, and `CLEAR_TH` at 40 to consider air as actually clear. This is called *hysteresis* — the dead zone between 40 and 80 prevents the robot from flickering back and forth when a reading bounces around the threshold. It's the same trick a thermostat uses.

Then we precompute the number of ticks for each motion: `REVERSE_STEPS` of 22 for the back-up, a 45-degree REORIENT step count of about 16, and — the interesting part — *per-robot* turn step counts. Robot1 turns 90 degrees, robot2 turns 105, robot3 turns 120. This is the *symmetry-breaking* piece. If all three robots had identical turns, two of them could collide and bounce off in mirror image and just re-collide forever. Different turn amounts guarantee they diverge after one collision. This is a real swarm-robotics technique.

---

### CODE PART 4 — Per-Robot Configuration & State Labels (Zain)

**[ZAIN]**
Now this is one of my favorite parts of the architecture. All three robots run the exact same Python file — but they behave differently because of this `ROBOT_CONFIG` dictionary. At startup, each robot calls `robot.getName()` and looks itself up in the dict. One file, three personalities.

Each robot's config has four fields. `init_dir` and `init_steps` give the small startup turn that points the robot east — robot1 turns 22.6 degrees left, robot2 turns 20 degrees right, robot3 only turns 2 degrees right. `avoid_default` is the fallback dodge direction when an obstacle is dead-on — and notice robot1 and robot2 have *opposite* defaults, so if they ever face each other head-on they resolve to opposite sides instead of locking up. And `avoid_turn_steps` is that symmetry-breaking turn duration we just talked about.

Below the config we declare the five state labels — `INIT_TURN`, `DRIVE`, `REVERSE`, `TURN`, `REORIENT`. The flow is: start in INIT_TURN to face east, then transition to DRIVE, which is where the robot spends about 99% of its life. When something happens — beacon detected or obstacle hit — it cycles through REVERSE, TURN, REORIENT, and back to DRIVE.

---

### CODE PART 5 — Robot Initialization (Yousif)

**[YOUSIF]**
This block is where the script actually wakes up the hardware. We instantiate the `Robot`, look up our name in the config, and then we go through three setup phases.

First, the motors. Webots motors have two modes — position mode, like a servo, and velocity mode, where you tell it "spin continuously at this rate." We want velocity mode, and the convention for that is to set the target position to *infinity* — that's `m.setPosition(float("inf"))`. Then we set initial velocity to zero.

Second, we loop through the eight IR sensors `ps0` to `ps7`, grab a handle to each one, and call `enable(TIME_STEP)` so they actually sample. If you forget to enable them, they return `NaN`. The four sensors we'll actually read in the main loop are ps0, ps1, ps6, and ps7 — those are the front-facing ones.

Third, the receiver. We grab it, enable it, and set its channel to 1 — same as the beacon's emitter. If those channels don't match, the signal never gets through.

---

### CODE PART 6 — Motion & Sensor Helpers (Zain)

**[ZAIN]**
These are the building blocks that the main loop will use over and over. The `wheels` function takes a left and right velocity and clamps both to the safe range — that's the classic `max-of-min` clamping trick. `spin_left` runs the left wheel backward and the right wheel forward, which makes the robot rotate counter-clockwise in place. `spin_right` does the opposite. That's standard differential drive.

Then we have the sensor helpers. `read_ps` returns the current values of all eight sensors as a list. `front_blocked` looks at the four front sensors and checks if any of them exceed the obstacle threshold of 80.

The interesting one is `choose_avoid_dir`. It compares how blocked the *right front* is versus the *left front*. If the right side is more blocked by at least 20 units, it returns "left" — turn away from the obstacle. If the left side is more blocked, it returns "right." If the readings are within 20 of each other — meaning the obstacle is dead-on — it falls back to the robot's per-robot default. That 20-unit buffer is hysteresis again, so it doesn't act on sensor noise.

---

### CODE PART 7 — `classify_obstacle` (Yousif)

**[YOUSIF]**
This is the biggest function in the file, but I'll be upfront — it doesn't change the robot's behavior at all. It's pure flavor for the console log. It looks at three things: the *peak* sensor reading which tells us how close the obstacle is, the *symmetry* between the left and right front sensors which tells us how square-on the hit was, and the *side_max* which checks if there's stuff on the sides too — meaning a corner or a cluster.

It then runs through a decision tree. If the peak is over 2500 it's basically a wall — we label it either "ARENA BOUNDARY WALL," a "CORNER" version of that, or a "LARGE BASALT BOULDER" depending on the symmetry and side readings. Between 800 and 2500 it's a "VOLCANIC ROCK FORMATION" or a "GLANCING HIT." Between 200 and 800 it's a "LOOSE ROCK CLUSTER" or a "SMALL ROCK FRAGMENT." And below 200 it gets labeled as "METALLIC DEBRIS FIELD."

The robot's actual avoidance behavior is identical regardless of which label it printed — this function exists purely to make the log feel like a real Mars mission, with each obstacle type getting a dramatic flavor message in the console.

---

### CODE PART 8 — State Variables & Main Loop Top (Zain)

**[ZAIN]**
Now we get into the runtime. We initialize a few state variables — `state` starts as `INIT_TURN`, `phase_step` counts ticks within the current state, `avoid_dir` holds the current dodge direction, `obstacle_count` tallies collisions for the log, and we have two flags for the beacon: `beacon_found` is persistent — has the robot *ever* heard the beacon — and `beacon_signal_active` will be set per-tick — is the beacon being heard *right now*. The persistent flag is what unlocks the periodic NASA uplink prints, and the per-tick flag is what triggers a re-establish-uplink message.

Inside the main loop, every tick we read the sensors, get the simulation time, and then we *drain the receiver queue*. Every message the beacon broadcasts gets queued up, so we have to read each one and call `nextPacket` to discard it — otherwise the queue grows forever. There's a `try/except` around `getString()` because newer Webots uses `getString` and older Webots uses `getData().decode()`, and we want this code to work on both versions.

---

### CODE PART 9 — NASA Uplink & `INIT_TURN` (Yousif)

**[YOUSIF]**
After we drain the queue, we have the NASA uplink check. This one sits *outside* the state machine — it fires on its own timer regardless of what the robot is doing. The condition is: the beacon has been found *and* at least 600 simulated seconds — that's 10 minutes — have passed since the last uplink. When that fires, it prints the dramatic mission-control block — elapsed time, telemetry uplink active, beacon coordinates transmitted — and updates `last_nasa_time`. The 10-minute clock starts when the beacon is first found, not when the simulation starts.

Then we enter the state machine ladder. The first state is `INIT_TURN`. The robot spins slowly — at the slow `INIT_TURN_SPEED` of 2.0 — for the per-robot number of ticks defined in its config, which points it roughly east. Once `phase_step` reaches the target, we print "heading locked," and transition to `DRIVE`.

This is the *timed-state pattern* you'll see used in every state except DRIVE — count up to a target, do the motion each tick, and when you hit the target, transition to the next state. Only DRIVE breaks this pattern, because DRIVE doesn't end on a clock — it ends on a sensor event.

---

### CODE PART 10 — The `DRIVE` State (Zain)

**[ZAIN]**
DRIVE is the heart of the controller. The robot spends about 99% of its time here. There are three branches, evaluated in priority order.

Priority one is *beacon signal active*. Crucially, this is checked *before* the obstacle check, because the beacon sphere is also a physical object — its IR returns would otherwise trigger `front_blocked`. Without this ordering, the robot would mistake the beacon for a small rock and never log a find. The first time the beacon is heard, we print the big celebration block — "BEACON FOUND," "RADIO SIGNAL CONFIRMED," "MARS BEACON SECURED" — flip `beacon_found` to True, and start the NASA uplink clock. On every subsequent re-detection, we just print a one-line "RE-ESTABLISHING UPLINK." Either way, the robot picks an avoid direction and transitions to REVERSE — meaning it backs *off* from the beacon so it can keep exploring instead of camping on it.

Priority two is *front blocked*. We pick a dodge direction, run `classify_obstacle` for the flavor label, increment the obstacle counter, look up a dramatic Mars-themed message based on the obstacle type, print it, and transition to REVERSE.

The third and default branch is to just cruise — both wheels at `DRIVE_SPEED` of 5.5, going straight forward. Stay in DRIVE until something happens.

---

### CODE PART 11 — `REVERSE`, `TURN`, `REORIENT` (Yousif)

**[YOUSIF]**
Once DRIVE has decided to dodge, the next three states execute the plan blindly — no sensor checks, no second-guessing. We call this *fire-and-forget* — the smart decision was already made in DRIVE.

`REVERSE` runs both wheels backward at about 55% of drive speed for 22 ticks — that's roughly 700 milliseconds of backing up. This gives clearance for the spin without grinding the wheels against whatever we just hit.

`TURN` is the big rotation. It spins in the chosen direction at the fast `TURN_SPEED` of 3.0 for the *per-robot* number of ticks. Robot1 spins 21 ticks — that's 90 degrees. Robot2 spins 24 — that's 105 degrees. Robot3 spins 28 — that's 120 degrees. That's the symmetry-breaking we talked about earlier, executed.

`REORIENT` is a partial counter-spin. It spins in the *opposite* direction at the slow `INIT_TURN_SPEED` for 16 ticks — about 45 degrees. So robot1 nets a 45-degree heading change after the full dodge, robot2 nets 60, robot3 nets 75. None of them drift back to perfectly east, which means each robot wanders a unique path through the arena over time.

After REORIENT we transition back to DRIVE, and the cycle is ready to start again.

A full dodge — REVERSE plus TURN plus REORIENT — takes about 60 ticks, or 2 seconds of real time. Then the robot's back to cruising.

*[End of code walkthrough — return to slides.]*

---

## SLIDE 10 — Testing (T07 / T12 / T13)

**[ZAIN]**
Now back to the slides. During development we hit a few specific failures worth flagging.

T07: we initially tried to detect the beacon using the IR sensors alone. The beacon is a small physical sphere, so when the robot drove into it, the IR peak read about 88 — which our classifier labeled as "METALLIC DEBRIS FIELD" rather than recognizing the beacon. No beacon-found message ever printed. The fix was to abandon IR for beacon detection entirely and use the e-puck's *built-in* radio receiver instead — enable it, set its channel to 1, and check `receiver.getString() == "BEACON"` every step.

T12 and T13: we then tried adding the receiver to the e-puck through the `extensionSlot` and the `turretSlot` in the world file. Both attempts broke because the e-puck PROTO already has a receiver device built in — we were defining a duplicate. The fix was simple: just call `robot.getDevice("receiver")` in the controller and enable it. No world-file change required.

---

## SLIDE 11 — Observations and Results

**[YOUSIF]**
For results, we observed five distinct behaviors during the simulation. The *startup behavior* — each robot doing its small unique initial turn to face east. *Obstacle encounter and classification* — the dramatic Mars-themed log messages firing whenever a robot bumped into a wall or rock. *Beacon detection* — the moment one of the robots gets within half a meter of the beacon sphere and the celebration block prints. *Continued exploration and NASA uplink* — the periodic 10-minute telemetry messages firing for any robot that has found the beacon. And the *terrain visual outcome* — three robots fanning out across the Mars surface, each tracing a distinct path because of the symmetry-breaking parameters.

---

## SLIDE 12 — Challenges Encountered & Key Learnings

**[ZAIN]**
A few real challenges came up. The robots circled instead of navigating in our early versions — that's what pushed us into the `INIT_TURN` plus per-robot heading correction. We had wall and corner entrapment problems, which is why the avoidance turn is such a big angle — 90 to 120 degrees — rather than a small one. The beacon misclassification by IR is what we just talked about in T07. And the e-puck PROTO device conflicts came from T12 and T13.

**[YOUSIF]**
The key takeaways for us were three things. First, the simulation environment imposes hardware constraints we didn't expect — the e-puck PROTO already includes a lot of devices, and you have to work with what's there rather than declaring duplicates. Second, the state machine was the right architecture — it kept the code organized and made debugging much easier than a single tangled if-else block. And third, combining open-loop timed control — the fixed REVERSE, TURN, REORIENT sequence — with reactive obstacle avoidance — the IR-driven branch in DRIVE — turned out to be very effective. The reactive part decides *what* to do, the timed part executes it without second-guessing.

---

## SLIDE 13 — Future Work

**[YOUSIF]**
There are a few directions we'd like to take this further. Adding GPS or wheel odometry would let the robots correct their drift over longer simulation runs and navigate more precisely. Adding inter-robot communication — so they can share obstacle positions or beacon coordinates — would let us move from independent fan-out to true coordinated swarm behavior.

**[ZAIN]**
We'd also like to add dynamic obstacle avoidance — moving rocks or debris — to stress-test the reactive layer. Expanding the arena and adding more robots would let us better demonstrate the coverage and redundancy advantages that swarm exploration provides. And eventually we'd love to take this off the simulator and onto physical hardware to validate that the symmetry-breaking and state machine design work in the real world.

---

## SLIDE 14 — References

**[ZAIN]**
We've listed our references on this slide — work from JPL on the CADRE technology demonstration, foundational swarm robotics papers from Dorigo and his collaborators, and the Webots simulator paper from Cyberbotics. We'll leave this up briefly for anyone who wants to read further.

---

## SLIDE 15 — Thank You

**[YOUSIF]**
That's our presentation. Thank you for listening — we're happy to take any questions.

**[ZAIN]**
Yes, please ask away.

---

## QUICK REFERENCE — Code Section Split

| Section | Topic | Presenter |
|---------|-------|-----------|
| 1 | `beacon_controller.py` | **Yousif** |
| 2 | Imports & Hardware Constants | **Zain** |
| 3 | Sensor Thresholds & Avoidance Timing | **Yousif** |
| 4 | Per-Robot Configuration & State Labels | **Zain** |
| 5 | Robot Initialization | **Yousif** |
| 6 | Motion & Sensor Helpers | **Zain** |
| 7 | `classify_obstacle` | **Yousif** |
| 8 | State Variables & Main Loop Top | **Zain** |
| 9 | NASA Uplink & `INIT_TURN` | **Yousif** |
| 10 | The `DRIVE` State | **Zain** |
| 11 | `REVERSE`, `TURN`, `REORIENT` | **Yousif** |

**Total split:** Yousif — 6 sections; Zain — 5 sections.

---

## QUICK REFERENCE — Slide Split

| Slide | Topic | Lead |
|-------|-------|------|
| 1 | Title | **Yousif** opens, **Zain** intros himself |
| 2 | The Problem | Both (Yousif then Zain) |
| 3 | Objectives | Both (Zain then Yousif then Zain) |
| 4 | Tools and Technologies | Both (Yousif then Zain) |
| 5 | Block Diagram | **Zain** |
| 6 | Design Methodology | Both (Yousif then Zain) |
| 7 | Component Table | Both (Yousif then Zain) |
| 8 | Flowchart | **Yousif** |
| 9 | Live Demo Intro | **Zain** |
| — | *Code walkthrough — alternating, see above* | — |
| 10 | Testing | **Zain** |
| 11 | Observations and Results | **Yousif** |
| 12 | Challenges & Key Learnings | Both (Zain then Yousif) |
| 13 | Future Work | Both (Yousif then Zain) |
| 14 | References | **Zain** |
| 15 | Thank You | Both |
