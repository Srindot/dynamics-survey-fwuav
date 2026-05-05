"""Trace exactly what happens to a single strip during one flap cycle."""
import numpy as np
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.simulator.orchestrator import Orcehstrator

sim = Orcehstrator(wing_span=1.4, num_strips=10)

# Run a few steps and print strip-level data
dt = 0.001
print("time, flap_rate, strip0_vz, strip0_airspeed_z, strip0_Fz, body_pitch")
for step in range(500):  # 0.5s
    t = step * dt
    sim.actuator.push_time(t)
    sim.down_pass(t)
    sim.aerodynamics_call()
    
    strip0 = sim.strips_l[0]
    vel = strip0.pull_airspeed
    state = strip0.pull_state
    wrench = strip0.pull_wrench
    flap = sim.actuator.pull_flap
    
    if step % 25 == 0:  # every 25ms
        print(f"{t:.3f}, {flap[1]:.2f}, {state[2]:.4f}, {vel[2]:.4f}, {wrench[2]:.4f}, {sim.body.pull_pose[1]:.4f}")
    
    sim.up_pass(dt)
