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

# Generic Base frame: parent for all the coordinate frames below
class Base_Frame:

    def __init__(self): 

        # Vector Definition

        # Pose Vector = position(px, py, pz), orientationy(ox, oy, oz)
        self.pose = np.zeros(6)

        # State Vector = velocity(vx, vy, vz), angulr rates(rx, ry, rz)
        self.state = np.zeros(6)

        # Accelertion Vector:  accelertion(ax, ay, az) + angular acceleration(wx, wy, wz)
        self.acc = np.zeros(6)

        # Force Vector = force(fx, fy, fz), moment(mx, my, mz)
        self.wrench = np.zeros(6)

    # Push Vectors
    def push_pose(self, pose_in):
        self.pose = pose_in 

    def push_state(self, state_in):
        self.state = state_in    

    def push_acc(self,acc_in):
        self.acc = acc_in

    def push_wrench(self, wrench_in):
        self.wrench = wrench_in

    # Pull state
    @property
    def pull_pose(self):
        return self.pose.copy()

    @property
    def pull_state(self):
        return self.state.copy() 

    @property
    def pull_acc(self):
        return self.acc.copy()

    @property
    def pull_wrench(self):
        return self.wrench.copy() 

    @property
    def pull_airspeed(self):
        """ 
        pull_airspeed: Returns the local airspeed vector (u, v, w). 
        Mathematically, this is exactly opposite to the frame's kinematic linear velocity.
        """
        return -self.state[:3].copy()



# Frame: Inertial
class Inertial_Frame(Base_Frame):

    def __init__(self): 
        super().__init__()

        # Define gravity
        self.gravity = np.array([0.0, 0.0, 9.81])
    
    @property
    def pull_gravity(self):
        return self.gravity.copy()


# Frame: Body  
class Body_Frame(Base_Frame):

    def __init__(self):

        # Call Parent's Init
        super().__init__()

        # Body Properties

        # Mass
        self.mass = 0.553  # kg

        # Inertial Matrix about COM
        # Converrsion Factor
        conv = 1e-9 
        self.I = np.array([
            [1.766E+07 * conv, -15753.614 * conv,  1.975E+06 * conv], # X-axis row
            [-15753.614 * conv, 2.159E+07 * conv,  1706.565 * conv ], # Y-axis row
            [1.975E+06 * conv,  1706.565 * conv,   3.812E+07 * conv]  # Z-axis row
        ])

    @property
    def pull_inertia(self):
        return self.I.copy()
        


# Frame: Winghinge
class Winghinge_Frame(Base_Frame):
    def __init__(self, side="left"):

        # Call Parent's Init
        super().__init__()
        
        # Winghinge (Frame) Properties

        # Side 
        self.side = side
        sign = 1 if side == "left" else -1

        # Pose for Wing Hinge from Body COM
        self.pose[:3] = np.array([0.18097, sign * 0.01345, 0.01557])
 
        # Vectors Definition
        # Flapping Vector = flapping angle(fa), flapping rate (fr)
        self.flap = np.zeros(2)

    # Push Vector
    def push_flap(self, flap_in):
        self.flap = flap_in
            
    # Pull state
    @property
    def pull_flap(self):
        return self.flap.copy()



 # Frame: Wing_Strip
class Wing_Strip(Base_Frame):
    def __init__(self, chord, strip_width, chordwise_cop, y_offset, spanwise_bend):

        # Call Parent's Init
        super().__init__()

        # Body Properties
        # Chord Length for each Strip
        self.chord = chord

        # Strip Width Spanwise
        self.strip_width = strip_width
        
        # Local offset along the wingspan
        self.y_offset = y_offset
        
        # Local Center of Pressure offset from leading edge
        self.chordwise_cop = chordwise_cop
        
        # Structural Mass of this segment
        self.strip_mass = 0.0
        
        # Translation of each strip
        self.pose[:3] = np.array([chordwise_cop, y_offset, spanwise_bend])
   
    @property 
    def pull_aero(self):
        return np.array([self.chord, self.strip_width, self.pose[0], self.pose[1]])


# Frame : Tail Hinge 
class Tailhinge_Frame(Base_Frame):
       
    def __init__(self):

         # Call Parent's Init
        super().__init__()
 
        # Tail Hinge position from the BodyFrame Com
        self.pose[:3] = np.array([-0.20148, 0.00054, 0.02835])

        # Tail Control Surface Deflection = (Elevator de, Rudder dr)
        self.deflection = np.zeros(2)
        
    def push_deflection(self, def_in):
        self.deflection = def_in
        
    @property
    def pull_deflection(self):
        return self.deflection.copy()
   

# Frame : Tail COP 
class TailCOP_Frame(Base_Frame):
       
    def __init__(self):

         # Call Parent's Init
        super().__init__()
 
        # Tail COP position from the BodyFrame Com
        self.pose[:3] = np.array([-0.1, 0.0, 0.0])

        # Tail Dimensions = ( chord, span, rudder height)
        self.shape = np.array([0.22, 0.38, 0.18])

    # Pull Data
    @property
    def pull_shape(self):
        return self.shape.copy()

   
