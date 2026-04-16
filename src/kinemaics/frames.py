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
        self.force = np.zeros(3)

        # Moments 
        self.moment = np.zeros(3)

        # Acceleration
        # Linear Accel(3) + Angular Accel(3)
        self.acc = np.zeros(6)

    # Push state 
    def push_state(self, linear_vel, angular_rate):
        self.state = np.concatenate([linear_vel, angular_rate])

    def push_forces(self, force_in, moment_in):
        self.force = force_in
        self.moment = moment_in

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
    def forces(self):
        return self.force.copy()

    @property
    def moments(self):
        return self.moment.copy()

    @property
    def linear_acceleration(self):
        return self.acc[:3].copy()

    @property
    def angular_acceleration(self):
        return self.acc[3:6].copy()

# Frame: Winghinge
class Winghinge_Frame:
    def __init__(self, actuation_system, side = "left"):

        # Side 
        self.side = side
        self.sign = 1 if self.side == "left" else -1

        # Offset from the com for the hinge location 
        self.offset_from_com = np.array([0.18097, self.sign * 0.01345, 0.01557])
        self.incidence_angle = np.radians(5.0)  

        # State  ( Vel(vx, vy, vz), Angular Rate(rx, ry, rz), Flapping_Angle, Flapping_Rate)
        self.state = np.zeros(8)

        # Forces and Moments
        self.force = np.zeros(3)
        self.moment = np.zeros(3)

        # actution system instance  
        self.actuation_system = actuation_system

    def push_state(self, vel_in, rate_in):

       # Function call from actuation_system : returns a np.array
       flapping_state = self.actuation_system.get_flapping_state()

       # Update the state
       self.state[0:3] = vel_in
       self.state[3:6] = rate_in
       self.state[6:8] = flapping_state

    def push_forces(self, force_in, moment_in):
        self.force = force_in
        self.moment = moment_in

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
    def forces(self):
        return self.force.copy()

    @property
    def moments(self):
        return self.moment.copy()

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

    # Push the Forces & Moments
    def push_forces(self, force_in, moment_in):
        self.force = force_in   
        self.moment= moment_in

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

        self.hinge = np.array([-0.3, 0.0, -0.1])

        self.tail_cop_offset = np.array([-0.1, 0.0, 0.0])

        self.state = np.zeros(6)

        self.force = np.zeros(3)

        self.moment = np.zeros(3)

        self.actuation_system = actuation_system
        

    def push_state(self, body_v, body_w):

        elev = self.actuation_system.get_elevator()
        rudd = self.actuation_system.get_rudder()

        hinge_pose = np.array([0.0, 0.0, 0.0])
        
        v_hinge = velocity_Transform_Body_to_Winghinge(
            body_v, body_w, hinge_pose, self.hinge_offset
        )
        w_hinge = body_w 

        cop_pose = np.array([0.0, elev, rudd])
        
        v_cop = velocity_Transform_Body_to_Winghinge(
            v_hinge, w_hinge, cop_pose, self.cop_offset
        )
        
        w_cop = rate_Transform_Body_to_Winghinge(
            w_hinge, cop_pose, 0.0
        )

        self.state = np.concatenate([v_cop, w_cop])

    def push_forces(self, force_in, moment_in):
        self.force = force_in
        self.moment = moment_in

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
To Do 
- Write the clas for Tail Frame
- Make sure the transfomration functions are aligining with the class frame
- Make sure the r_offset from teh class frame are correct with respect to the r_offset or leverarm in the transformation function.
- Write the super class to handle the up pass and down pass 
- write rigid body dynamics 
-  implement multiple aerodynamic model
"""
  
"""
2. Calculated Offsets for your ClassesThese are the exact vectors you should plug into your HingeFrame instances:A. Wing Hinge OffsetThe wings are actually in front of the CoM (since $-0.041$ is more positive than $-0.222$).Left Wing Offset: $[+0.18097, +0.01345, +0.01557]$Right Wing Offset: $[+0.18097, -0.01323, +0.01557]$B. Tail Hinge OffsetThe tail is significantly behind the CoM.Tail Offset: $[-0.20148, +0.00054, +0.02835]$

"""
