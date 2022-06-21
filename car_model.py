import numpy as np

class Model():

    def __init__(self, eta, m_b, m_c, g, r):
        #  v    = vehicle velocity (m/s)
        #  t    = time (sec)
        #  u    = gas pedal position (-50% to 100%)
        # Fp = ??
        
        self.eta = eta #0.25
        self.m_b = m_b #1 # mass of the ball
        self.m_c = m_c
        self.g = g #9.8 # gravity
        self.r = r #1 # radius of the pendulum

    def ode_angle(self, theta, t, u, Fp):
        dSTheta_dSt = [theta[1], (((Fp*u/self.m_c)*np.cos(theta[0])) - (self.eta*theta[1]*self.r)/self.m_b - (np.sin(theta[0])*self.g))]
        return dSTheta_dSt

    def ode_velocity(self, v, t, u, Fp):
        # inputs
        #  v    = vehicle velocity (m/s)
        #  t    = time (sec)
        #  u    = gas pedal position (-50% to 100%) 
        #Fp     # thrust parameter (N/%pedal)
        Cd = 0.24     # drag coefficient
        rho = 1.225  # air density (kg/m^3)
        A = 5.0      # cross-sectional area (m^2)
        m = self.m_c     # vehicle mass (g)
        # calculate derivative of the velocity
        dv_dt = (1.0/(m)) * (Fp*u - 0.5*rho*Cd*A*v**2)
        return dv_dt