"""Fly constant-speed legs for flow_cnn training data, then land.

Run vicon_recorder.py (flow_cnn) alongside it. Each leg goes out and back along
one axis at one speed, so the drone ends where it started. Only the steady middle
of a leg passes flow_cnn's filters (level, 1.2-2.0 m up), so legs are long enough
to leave room for speeding up and slowing down. Size LEG to fit the room.
"""

from jl_flight import mission

HEIGHT = 1.5  # m, inside flow_cnn's 1.2-2.0 m training band
LEG = 2.0  # m each way
SPEEDS = (0.3, 0.5)  # m/s; jl_mission clamps anything over 1.0


@mission("FlowData")
def flow_data(drone):
    drone.take_off(HEIGHT)
    drone.hover(2)
    for v in SPEEDS:
        drone.say(f"forward/back at {v} m/s")
        drone.forward(LEG, velocity=v)
        drone.back(LEG, velocity=v)
        drone.say(f"left/right at {v} m/s")
        drone.left(LEG, velocity=v)
        drone.right(LEG, velocity=v)
        drone.hover(1)
    drone.land()
