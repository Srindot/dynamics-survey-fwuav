import os
import csv
import numpy as np

class SimulationLogger:
    def __init__(self):
        self.history = []
        self.headers = [
            'time', 
            'pos_x', 'pos_y', 'pos_z',
            'roll', 'pitch', 'yaw',
            'vel_x', 'vel_y', 'vel_z',
            'p', 'q', 'r',
            'flap_angle', 'flap_rate',
            'elevator', 'rudder',
            'force_x', 'force_y', 'force_z',
            'moment_x', 'moment_y', 'moment_z'
        ]

    def log_step(self, t, sim):
        """
        Extracts snapshot of current physical variables from the Orchestrator.
        """
        # Body Pose & State
        pose = sim.body.pull_pose
        state = sim.body.pull_state
        
        # Actuation
        flap = sim.actuator.pull_flap
        tail = sim.actuator.pull_deflection
        
        # Wrench
        wrench = sim.body.pull_wrench
        
        record = [
            t,
            pose[0], pose[1], pose[2],
            pose[3], pose[4], pose[5],
            state[0], state[1], state[2],
            state[3], state[4], state[5],
            flap[0], flap[1],
            tail[0], tail[1],
            wrench[0], wrench[1], wrench[2],
            wrench[3], wrench[4], wrench[5]
        ]
        
        self.history.append(record)

    def save_to_csv(self, filename="plot/simulation_data.csv"):
        """
        Exports the logged history to a CSV file.
        """
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)
            writer.writerows(self.history)
        print(f"Data successfully exported to {filename}")
