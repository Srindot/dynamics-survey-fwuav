# BIONIC FLAPPING-WING UAV SIMULATOR: COMPLETE ARCHITECTURAL SPECIFICATION

## I. MODULAR SYSTEM STRUCTURE (The "Anatomy")

### 1. CORE MATH ENGINE (`quaternion_transform.py`)
- **Functionality:** Pure mathematical rotations using Quaternions and Rotation Vectors.
- **Key Tools:**
  - `transform_vector()`: Rotates vectors between frames.
  - `transform_rate()`: Rotates angular rates between frames.

### 2. COORDINATE FRAME DATA HOLDERS (`frames.py`)
- **Functionality:** Class-based storage for the physical state of every component.
- **Frames Included:**
  - `Inertial_Frame`: NED Global Position and Orientation.
  - `Body_Frame`: Mass, Inertia Matrix [I], and CoM Wrench (Force/Moment).
  - `Winghinge_Frame`: Flapping state, incidence angles, and hinge offsets.
  - `Wingstrip_Frame` (x20): Chord, width, and local flow velocity.
  - `Tail_Frame`: Elevator/Rudder state and aerodynamic center.

### 3. KINEMATIC & WRENCH PIPELINE (`frame_transform.py`)
- **Functionality:** Functional logic to move data between the classes.
- **Functions:**
  - `velocity_Transform_...()`: (Down-Pass) Moves velocity from Body -> Hinge -> Strips.
  - `wrench_Transform_...()`: (Up-Pass) Moves Forces/Moments from Strips -> Hinge -> Body.

### 4. AERODYNAMIC KERNEL (`aerodynamics.py` - FUTURE)
- **Functionality:** Blade Element Theory (BET) engine.
- **Logic:** Converts strip `[u, v, w]` into Lift/Drag forces.

### 5. DYNAMICS SOLVER (`dynamics.py` - FUTURE)
- **Functionality:** Rigid body physics.
- **Logic:** Newton-Euler 6-DOF solver to find translational and rotational accelerations.

---

## II. SIMULATION EXECUTION LOOP (The "Physiology" - Per Time Step `dt`)

### STEP 1: THE DOWN-PASS (Kinematic Propagation)
- **[Body -> WingHinge]:** Calculate Hinge velocity using Body rate and `[r_offset]`.
- **[WingHinge -> Wingstrip]:** Distribute velocity to all 20 strips. Include local flapping rate (e.g., 5Hz) at the hinge to find the effective instantaneous airspeed at each wing segment.
- **[Body -> Tail]:** Propagate velocity to the tail hinge and then to the Tail CoP.

### STEP 2: THE AERO BLOCK (Force Generation)
- **Wingstrips:** For each of the 20 Wingstrips, calculate Pitch/Yaw flow angles (Alpha/Beta), compute Lift/Drag, and calculate the Pitching Moment.
- **Tail:** Compute Control Surface forces based on current Elevator/Rudder deflection angles.

### STEP 3: THE UP-PASS (Wrench Accumulation)
- **[Wingstrip -> WingHinge]:** Transform local strip forces into the Hinge Wrench (Adds `r x F` torque).
- **[Mirroring]:** Mirror Left Wing forces to Right Wing by surgically flipping Y-axis signs.
- **[WingHinge -> Body]:** Sum all 40 strips and translate the total load to the Body Center of Mass (CoM).
- **[Tail -> Body]:** Move the Tail Wrench to the Body CoM utilizing the calculated body-to-tail offsets.

### STEP 4: RIGID BODY DYNAMICS (The Solver)
- Sum ALL accumulated Wrench loads (Wings + Tail + Gravity) operating on the Body CoM.
- Solve Newton-Euler rigid body equations:
  - **Linear:** Acceleration = $\sum F / \text{Mass}$
  - **Angular:** Angular Accel ($\alpha$) = $I^{-1} \times (\sum M - \omega \times (I \times \omega))$

### STEP 5: STATE INTEGRATION (The Final Update)
- **Update Velocities:** Vel = Vel + (Accel * dt)
- **Update Pose:** Position/Orientation = Pose + (Vel * dt) (or via RK4 integration)
- Push the new physical states into the `frames.py` classes, ready for the next iteration step.
