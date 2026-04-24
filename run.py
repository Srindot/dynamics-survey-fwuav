import os
import sys

# Ensure src module can resolve locally
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.simulator.simulation import simulation

if __name__ == "__main__":
    print("==== BIONIC FLAPPING WING UAV SIMULATOR ====")
    
    # Configure simulation parameters
    DT = 0.001
    TOTAL_TIME = 2.0  # seconds
    ITERATIONS = int(TOTAL_TIME / DT)
    
    # Orchestrator parameters
    WING_SPAN = 1.4
    NUM_STRIPS = 10
    
    # Boot Sequence
    sim = simulation(dt=DT, iterations=ITERATIONS, wing_span=WING_SPAN, num_strips=NUM_STRIPS)
