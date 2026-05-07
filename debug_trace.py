"""
Debug Trace: Runs 3 timesteps of the simulation and prints EVERY intermediate variable
to identify the exact source of negative body_vel_x.
"""
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
np.set_printoptions(precision=6, suppress=True)

from src.simulator.orchestrator import Orcehstrator
from src.kinemaics.frame_transform import (
    state_Transform_Body_to_Winghinge,
    state_Transform_Winghinge_to_Wingstrip,
    wrench_Transform_Wingstrip_to_Winghinge,
    wrench_Transform_Winghinge_to_Body,
    wrench_Transform_TailCOP_to_Tailhinge,
    wrench_Transform_Tailhinge_to_Body,
    sum_wing_strip_wrenches,
    mirror_wing_wrench,
    state_Transform_Body_to_Inertial,
)
from src.rigid_body_dynamics.dynamics import sum_body_wrenches, calculate_state_derivative
from src.kinemaics.state_integrator import integrator

DT = 0.0001
sim = Orcehstrator(wing_span=1.4, num_strips=10)

# Run for a few macro-steps (each = 100 micro-steps to reach t=0.01s)
for macro in range(20):  # 20 macro steps = t=0.0 to t=0.02s
    t = macro * 100 * DT  # Print every 0.01s
    
    # Run 100 micro-steps silently
    for micro in range(100):
        t_now = (macro * 100 + micro) * DT
        
        sim.down_pass(t_now)
        sim.aerodynamics_call()
        sim.up_pass(DT)
    
    # PRINT DIAGNOSTICS after 100 micro-steps
    t_end = (macro + 1) * 100 * DT
    print(f"\n{'='*80}")
    print(f"TIME: {t_end:.4f}s")
    print(f"{'='*80}")
    
    # Body state
    pose = sim.body.pull_pose
    state = sim.body.pull_state
    wrench = sim.body.pull_wrench
    print(f"\n--- BODY ---")
    print(f"  pose    = {pose}")
    print(f"  state   = {state}")
    print(f"  wrench  = {wrench}")
    print(f"  vel_x   = {state[0]:+.8f}")
    print(f"  pitch   = {pose[4]:+.8f} rad = {np.rad2deg(pose[4]):+.4f} deg")
    
    # Actuation
    flap = sim.actuator.pull_flap
    print(f"\n--- ACTUATION ---")
    print(f"  flap_angle = {flap[0]:+.6f} rad  flap_rate = {flap[1]:+.6f} rad/s")
    print(f"  {'DOWNSTROKE' if flap[1] < 0 else 'UPSTROKE'}")
    
    # Wing strip 5 (mid-span)
    s = sim.strips_l[5]
    print(f"\n--- STRIP 5 (mid-span) ---")
    print(f"  state      = {s.pull_state}")
    print(f"  airspeed   = {s.pull_airspeed}")
    print(f"  vel[0]     = {s.pull_airspeed[0]:+.8f}")
    print(f"  vel[2]     = {s.pull_airspeed[2]:+.8f}")
    print(f"  alpha_geom = {getattr(s, 'alpha_geom', 'N/A')}")
    print(f"  epsilon_ce = {getattr(s, 'epsilon_ce', 'N/A')}")
    print(f"  alpha_e    = {getattr(s, 'alpha_e', 'N/A')}")
    print(f"  F_tran_x   = {getattr(s, 'F_tran_x', 'N/A')}")
    print(f"  F_tran_z   = {getattr(s, 'F_tran_z', 'N/A')}")
    print(f"  F_tot_x    = {getattr(s, 'F_tot_x', 'N/A')}")
    print(f"  F_tot_z    = {getattr(s, 'F_tot_z', 'N/A')}")
    print(f"  strip wrench = {s.pull_wrench}")
    
    # Wrench transformations
    wrench_hinge_l = sum_wing_strip_wrenches(sim.strips_l)
    flap_vector = sim.actuator.pull_flap
    wrench_body_l = wrench_Transform_Winghinge_to_Body(
        wrench_hinge_l, sim.winghinge_l.pull_pose, flap_vector)
    wrench_body_r = mirror_wing_wrench(wrench_body_l)
    
    print(f"\n--- WRENCH CHAIN ---")
    print(f"  hinge_l wrench  = {wrench_hinge_l}")
    print(f"  hinge_l pose    = {sim.winghinge_l.pull_pose}")
    print(f"  body_l wrench   = {wrench_body_l}")
    print(f"  body_r wrench   = {wrench_body_r}")
    print(f"  WING body_Fx    = {wrench_body_l[0] + wrench_body_r[0]:+.8f}")
    
    # Tail
    tail_wrench = sim.tailcop.pull_wrench
    print(f"\n--- TAIL ---")
    print(f"  tailcop airspeed = {sim.tailcop.pull_airspeed}")
    print(f"  tailcop wrench   = {tail_wrench}")
    
    tail_vector = sim.actuator.pull_deflection
    wrench_tailhinge = wrench_Transform_TailCOP_to_Tailhinge(
        sim.tailcop.pull_wrench, sim.tailcop.pull_pose, tail_vector)
    wrench_body_tail = wrench_Transform_Tailhinge_to_Body(
        wrench_tailhinge, sim.tailhinge.pull_pose)
    print(f"  tail body wrench = {wrench_body_tail}")
    print(f"  TAIL body_Fx     = {wrench_body_tail[0]:+.8f}")
    
    # Body aero
    body_aero = sim.body.pull_wrench
    print(f"\n--- BODY AERO ---")
    print(f"  body drag wrench = {body_aero}")
    print(f"  BODY body_Fx     = {body_aero[0]:+.8f}")
    
    # TOTAL
    total_Fx = wrench_body_l[0] + wrench_body_r[0] + wrench_body_tail[0] + body_aero[0]
    total_My = wrench_body_l[4] + wrench_body_r[4] + wrench_body_tail[4] + body_aero[4]
    print(f"\n--- TOTAL ON BODY ---")
    print(f"  TOTAL Fx = {total_Fx:+.8f} N")
    print(f"  TOTAL My = {total_My:+.8f} Nm (pitch moment)")
    
    # Gravity contribution
    from src.rigid_body_dynamics.dynamics import calculate_gravity_force
    f_grav = calculate_gravity_force(pose, sim.body.mass)
    print(f"\n--- GRAVITY ---")
    print(f"  gravity body = {f_grav}")
    print(f"  gravity Fx   = {f_grav[0]:+.8f} (from body pitch)")
