import numpy as np

class Kinematics:
    def __init__(self, mass, Jx, Jy, Jz, Jxz):
        """
        Initialize the 12-DoF kinematic parameters based on Beard 2025.
        """
        self.mass = mass
        self.Jx = Jx
        self.Jy = Jy
        self.Jz = Jz
        self.Jxz = Jxz
        
        # Calculate Gamma variables for rigid body dynamics
        Gamma = Jx * Jz - Jxz**2
        self.Gamma1 = Jxz * (Jx - Jy + Jz) / Gamma
        self.Gamma2 = (Jz * (Jz - Jy) + Jxz**2) / Gamma
        self.Gamma3 = Jz / Gamma
        self.Gamma4 = Jxz / Gamma
        self.Gamma5 = (Jz - Jx) / Jy
        self.Gamma6 = Jxz / Jy
        self.Gamma7 = ((Jx - Jy) * Jx + Jxz**2) / Gamma
        self.Gamma8 = Jx / Gamma

        # State initialization: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
        self.state = np.zeros(12)

    def compute_derivatives(self, state, forces_moments, g=9.81):
        """
        Computes the state derivatives based on the 12-DoF rigid body equations.
        Requires forces_moments array: [fx, fy, fz, l, m, n] in body frame
        """
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state
        fx, fy, fz, l, m, n = forces_moments

        # 1. Position Derivatives
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_th, s_th, t_th = np.cos(theta), np.sin(theta), np.tan(theta)
        c_psi, s_psi = np.cos(psi), np.sin(psi)

        dpn = c_th*c_psi*u + (s_phi*s_th*c_psi - c_phi*s_psi)*v + (c_phi*s_th*c_psi + s_phi*s_psi)*w
        dpe = c_th*s_psi*u + (s_phi*s_th*s_psi + c_phi*c_psi)*v + (c_phi*s_th*s_psi - s_phi*c_psi)*w
        dpd = -s_th*u + s_phi*c_th*v + c_phi*c_th*w

        # 2. Velocity Derivatives (including Gravity based on Beard 2025)
        du = r*v - q*w - g*s_th + fx / self.mass
        dv = p*w - r*u + g*c_th*s_phi + fy / self.mass
        dw = q*u - p*v + g*c_th*c_phi + fz / self.mass

        # 3. Angle Derivatives
        dphi = p + q*s_phi*t_th + r*c_phi*t_th
        dtheta = q*c_phi - r*s_phi
        dpsi = q*s_phi/c_th + r*c_phi/c_th

        # 4. Angular Rate Derivatives
        dp = self.Gamma1*p*q - self.Gamma2*q*r + self.Gamma3*l + self.Gamma4*n
        dq = self.Gamma5*p*r - self.Gamma6*(p**2 - r**2) + m / self.Jy
        dr = self.Gamma7*p*q - self.Gamma1*q*r + self.Gamma4*l + self.Gamma8*n

        return np.array([dpn, dpe, dpd, du, dv, dw, dphi, dtheta, dpsi, dp, dq, dr])

    def flapping_kinematics(self, t, A, f):
        """
        Kinematics for the flapping wing parameters.
        Using Mao 2024 eq 5 assumption: harmonic flapping.
        g(f,t) = cos(2*pi*f*t) -> common assumption for symmetric stroke
        Returns lam, omega_lambda, dot_omega_lambda
        """
        # Time-derivative term for flapping stroke frequency
        # Normal wing beat function: lambda(t) = (A/2) * cos(2*pi*f*t)
        omega = 2 * np.pi * f
        
        # Stroke angle
        lam = (A / 2) * np.cos(omega * t)
        
        # Eq 5: omega_lambda = (A/2) * d/dt(g(f,t))
        omega_lambda = -(A / 2) * omega * np.sin(omega * t)
        
        # Derivative of the stroke velocity
        dot_omega_lambda = -(A / 2) * (omega**2) * np.cos(omega * t)
        
        return lam, omega_lambda, dot_omega_lambda
