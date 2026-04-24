import numpy as np

class ActuationSystem:
    def __init__(self, flap_freq=10.0, flap_amplitude=np.deg2rad(60.0), elevator=0.0, rudder=0.0):
        """
        ActuationSystem: Generates motor driving parameters and servo angles.
        Default: 10 Hz flapping, 60-degree amplitude. Tail servos starting at 0.0.
        """
        self.freq = flap_freq
        self.amp = flap_amplitude
        
        # Internal States
        self.t = 0.0
        self.deflection = np.array([elevator, rudder])
        
    # --- PUSH METHODS ---
    def push_time(self, t_in):
        self.t = t_in

    def push_freq(self, freq_in):
        self.freq = freq_in

    def push_amp(self, amp_in):
        self.amp = amp_in

    def push_deflection(self, deflection_in):
        self.deflection = deflection_in

    # --- PULL METHODS ---
    @property
    def pull_freq(self):
        return self.freq

    @property
    def pull_amp(self):
        return self.amp

    @property
    def pull_deflection(self):
        return self.deflection.copy()
        
    @property
    def pull_flap(self):
        """
        Dynamically calculates and returns instantaneous flapping parameters 
        based on the internally stored simulation time (self.t).
        """
        omega = 2.0 * np.pi * self.freq
        
        # Sinusoidal flapping
        flap_angle = self.amp * np.sin(omega * self.t)
        flap_rate = self.amp * omega * np.cos(omega * self.t)
        flap_accel = -self.amp * (omega**2) * np.sin(omega * self.t)
        
        return np.array([flap_angle, flap_rate, flap_accel])
