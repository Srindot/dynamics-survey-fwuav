# frames.py: Contains all classes which govern the funcitions and parameters of each coordinate frames of the vehicle.
import numpy as np 

"""
frames.py: Contains Classes for properties of each coordinate frames in the vechicle
    Classes: 
        - Inertial_Frame
        - Body_Frame
        - Winghinge_Frame
        - Wingstrip_Frame
        - Tail_Frame
"""

# Frame: Inertial
class Inertial_Frame:

    def __init__(self): 

        # State Vector: [Position(NED): (x, y, z), Orientation(Rotation_Vector): (ox, oy, oz)]
        self.state = np.zeros(6)

    # Push state
    def push_state(self, position_in, orientation_in):

        # Update state vector
        self.state = np.concatenate([position_in, orientation_in])

    # Pull state  
    @property
    def position(self):
        return self.state[:3]

    @property
    def orientation(self):
        return self.state[3:6]


# Frame: Body  
class Body_Frame:

    def __init__(self):

        # Converrsion Factor
        conv = 1e-9 

        # Mass
        self.mass = 0.553  # kg

        # Inertial Matrix about COM
        self.I = np.array([
            [1.766E+07 * conv, -15753.614 * conv,  1.975E+06 * conv], # X-axis row
            [-15753.614 * conv, 2.159E+07 * conv,  1706.565 * conv ], # Y-axis row
            [1.975E+06 * conv,  1706.565 * conv,   3.812E+07 * conv]  # Z-axis row
        ])

        # State Vector
        self.state = np.zeros(6)

        # Force  
        self.forces = np.zeros(3)

        # Moments 
        self.moments = np.zeros(3)

        # Acceleration
        # Linear Accel(3) + Angular Accel(3)
        self.acc = np.zeros(6)

    # Push state 
    def push_state(self, linear_vel, angular_rates):
        self.state = np.concatenate([linear_vel, angular_rates])

    def push_forces(self, forces_in, moments_in):
        self.forces = forces_in
        self.moments = moments_in

    def update_accelerations(self, linear_acceleration_in, angular_acceleration_in):
        self.acc = np.concatenate([ linear_acceleration_in, angular_acceleration_in ])

    # Pull state
    @property
    def linear_velocity(self):
        return self.state[:3].copy()

    @property
    def angular_velocity(self):
        return self.state[3:6].copy()

    @property
    def force(self):
        return self.forces.copy()

    @property
    def moment(self):
        return self.moments.copy()

    @property
    def linear_acceleration(self):
        return self.acc[:3].copy()

    @property
    def angular_acceleration(self):
        return self.acc[3:6].copy()

# Frame: Winghinge
class Winghinge_Frame:
    def __init__(self, side="left", actuation_system):

        # Side 
        self.side = side
        sign = 1 if side == "left" else -1

        # Offset from the com for the hinge location 
        self.offset_from_com = np.array([0.18097, sign * 0.01345, 0.01557])
        self.incidence_angle = np.radians(5.0)  

        # State  ( Vel(vx, vy, vz), Angular Rate(rx, ry, rz), Flapping_Angle, Flapping_Rate)
        self.state = np.zeros(8)
       
        # Forces and Moments
        self.forces = np.zeros(3)
        self.moments = np.zeros(3)

        # actution system instance  
        self.actuation_system = actuation_system

    def push_state(self, vel_in, rate_in):
        flapping_angle = self.actuation_system.get_flap_angle()
        flapping_rate = self.actuation_system.get_flap_rate()
        self.state = np.concatenate([vel_in, rate_in, [ flapping_angle ], [ flapping_rate ]])

    def push_forces(self, forces_in, moments_in):
        self.forces = forces_in
        self.moments = moments_in

    # Pull The State & Data 
    @property
    def velocity(self):
        return self.state[:3].copy()

    @property
    def angular_rate(self):
        return self.state[3:6].copy()

    @property
    def flapping(self):
        return self.state[6:8].copy()

    @property
    def force(self):
        return self.forces 

    @property
    def moment(self):
        return self.moments

# Frame: Wing_Strip
class Wing_Strip:
    def __init__(self, chord, strip_width, chordwise_cop, y_offset, spanwise_bend):

        # Translation Array for each Strip
        self.translation = np.array([chordwise_cop, y_offset, spanwise_bend])

        # Chord Length for each Strip
        self.chord = chord

        # Strip Width Spanwise
        self.width = strip_width
        
        # State [Linear Velocity (u, v, w), Angular Rates (p, q, r)]
        self.state = np.zeros(6)

        # Forces & Moments in each Axis
        self.force = np.zeros(3)
        self.moment = np.zeros(3)

    # Push the State
    def push_state(self, vel_in, rate_in):

        self.state[0:3] = vel_in
        self.state[3:6] = rate_in 

    # Pull the State & Data
    @property
    def velocity(self):
        return self.state[:3].copy()

    @property
    def angular_rates(self):
        return self.state[3:6].copy()

    @property
    def forces(self):
        return self.force.copy()

    @property
    def moments(self):
        return self.moment.copy()

# Frame: Tail
class Tail_Frame:

    def __init__(self, actuation_system):

        # Tail Hinge position from the BodyFrame Com
        self.hinge = np.array([-0.20148, 0.00054, 0.02835])

        # Tail COP position from the Body COM
        self.tail_cop_offset = np.array([-0.1, 0.0, 0.0])
        
        # State Vector ( velocity(vx, vy, vz), angular_rate(rx, ry, rz))
        self.state = np.zeros(6)

        # Force Array
        self.force = np.zeros(3)

        # Moment Array  
        self.moment = np.zeros(3)
        
        # Actuation System Instance Pass down
        self.actuation_system = actuation_system
        
    # Push State & Data
    def push_state(self, velocity_in, rate_in):
        self.state[:3] = velocity_in
        self.state[3:6] = rate_in

    def push_forces(self, forces_in, moments_in):
        self.forces = forces_in
        self.moments = moments_in

    # Pull State & Data
    @property
    def velocity(self):
        return self.state[:3].copy()

    @property
    def angular_rate(self):
        return self.state[3:6].copy()
  
    @property
    def forces(self):
        return self.force.copy()

    @property
    def moments(self):
        return self.moment.copy()
"""
2. Calculated Offsets for your ClassesThese are the exact vectors you should plug into your HingeFrame instances:A. Wing Hinge OffsetThe wings are actually in front of the CoM (since $-0.041$ is more positive than $-0.222$).Left Wing Offset: $[+0.18097, +0.01345, +0.01557]$Right Wing Offset: $[+0.18097, -0.01323, +0.01557]$B. Tail Hinge OffsetThe tail is significantly behind the CoM.Tail Offset: $[-0.20148, +0.00054, +0.02835]$
"""
