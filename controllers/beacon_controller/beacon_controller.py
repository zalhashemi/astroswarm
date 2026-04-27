"""
Broadcasts a "BEACON" signal on channel 1 every simulation step.
Robots with a Receiver tuned to channel 1 can detect this signal
when within range, confirming beacon contact without needing GPS or camera.
"""

from controller import Robot

robot    = Robot()
emitter  = robot.getDevice("emitter")
TIME_STEP = 32

while robot.step(TIME_STEP) != -1:
    emitter.send("BEACON".encode("utf-8"))
