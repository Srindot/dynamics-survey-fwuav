import os
import csv
import numpy as np

class SimulationLogger:
    def __init__(self, num_strips=10):
        self.history = []
        self.num_strips = num_strips
        
        # Base headers
        self.headers = [
            'time', 
            'body_pos_x', 'body_pos_y', 'body_pos_z',
            'body_roll', 'body_pitch', 'body_yaw',
            'body_vel_x', 'body_vel_y', 'body_vel_z',
            'body_p', 'body_q', 'body_r',
            'flap_angle', 'flap_rate',
            'elevator', 'rudder',
            'body_force_x', 'body_force_y', 'body_force_z',
            'body_moment_x', 'body_moment_y', 'body_moment_z'
        ]
        
        # Add headers for left wing strips
        for i in range(num_strips):
            self.headers.extend([
                f'strip_l_{i}_alpha_geom',
                f'strip_l_{i}_epsilon_ce',
                f'strip_l_{i}_alpha_e',
                f'strip_l_{i}_v_in_sq',
                f'strip_l_{i}_F_tran_x',
                f'strip_l_{i}_F_tran_z',
                f'strip_l_{i}_F_rot_z',
                f'strip_l_{i}_F_add_z',
                f'strip_l_{i}_F_iner_z',
                f'strip_l_{i}_F_tot_x',
                f'strip_l_{i}_F_tot_z',
                f'strip_l_{i}_M_pitch'
            ])

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
        
        # Log Left Wing Strips Data (saved as custom attributes in aerodynamics.py)
        for strip in sim.strips_l:
            # We will pull these attributes safely if they exist
            record.extend([
                getattr(strip, 'alpha_geom', 0.0),
                getattr(strip, 'epsilon_ce', 0.0),
                getattr(strip, 'alpha_e', 0.0),
                getattr(strip, 'v_in_sq', 0.0),
                getattr(strip, 'F_tran_x', 0.0),
                getattr(strip, 'F_tran_z', 0.0),
                getattr(strip, 'F_rot_z', 0.0),
                getattr(strip, 'F_add_z', 0.0),
                getattr(strip, 'F_iner_z', 0.0),
                getattr(strip, 'F_tot_x', 0.0),
                getattr(strip, 'F_tot_z', 0.0),
                getattr(strip, 'M_pitch', 0.0)
            ])
            
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
