import numpy as np
from .orchestrator import Orcehstrator
from src.output.logger import SimulationLogger

def simulation(dt=0.01, iterations=1000, wing_span=1.4, num_strips=10):
    sim = Orcehstrator(wing_span=wing_span, num_strips=num_strips)
    logger = SimulationLogger()
    print("Initializing Modular Simulator Loop...")
    
    for step in range(iterations):
        t = step * dt
        
        # 1. Kinematics Down (Actuation polled natively)
        sim.down_pass(t)
        
        # 2. Force Math 
        sim.aerodynamics_call()
        
        # 3. Wrench Torque and Physics Up
        sim.up_pass(dt)
        
        # 4. Telemetry Logging
        logger.log_step(t, sim)
        
        if step % 100 == 0:
            print(f"Time: {t:.2f}s | Body Z Pos: {sim.body.pull_pose[2]:.4f} | Body Z Vel: {sim.body.pull_state[2]:.4f}")
            
    print("Simulation Complete.")
    logger.save_to_csv()
    return sim
