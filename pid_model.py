import numpy as np
import car_model
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class PID():
    def __init__(self, model, Kp, Ki, Kd): # Ki = Kc/taui -Kc * taud
        self.model = model
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd


    def run(self, tf, nsteps, sp, theta_omega0, noise):

        # tf: final time for simulation
        # nsteps: number of time steps
        # sp: vector of the reference angle (length == nsteps )
        # theta_omega0: initial condition for theta and omega [theta, omega]
        # noise: boolean representing if we want to use noise or not
        # Kp, Ki, Kd: PID parameters

        # Define the timesteps
        delta_t = tf/(nsteps)   # length of each timestep
        ts = np.linspace(0,tf,nsteps)

        # utility list for storing
        sp_store = np.zeros(nsteps)
        omega_store = np.zeros(nsteps)
        theta_store = np.zeros(nsteps)
        u_store = np.zeros(nsteps)
        theta_noisy_store = np.zeros(nsteps)

        # PID utilities
        sum_int = 0.0
        error = 0
        u = 0

        # store the speed to perform later some monitoring
        v0 = 0
        v_store = np.zeros(nsteps)

        numerator_D = 0
        # simulate with ODEINT
        for i in range(nsteps):
            
            v_store[i] = v0
            sp_store[i] = sp[i]
            omega_store[i] = theta_omega0[1] 
            theta_store[i] = theta_omega0[0]

            # I want to save the clear data but I want to compute the new u having some noise in the observations
            theta_omega0[0] = theta_omega0[0] + noise[i]

            theta_noisy_store[i] = theta_omega0[0]
            
            #prec_error = error
            error = sp[i] - theta_omega0[0]
            sum_int += error*delta_t
            
            if i > 0:
                numerator_D = (theta_omega0[0]-theta_store[i-1]) # with observed for the derivative kick
            #numerator_D = error - prec_error
            u = self.Kp*error + self.Ki * sum_int - self.Kd * (numerator_D/(delta_t))
            # Bounds for the controlled variable
            if u >= 100.0:
                u = 100.0
                sum_int = sum_int - error*delta_t
            if u <= -50.0:
                u = -50.0
                sum_int = sum_int - error*delta_t
            u_store[i] = u

            theta_omega = odeint(self.model.ode_angle,theta_omega0,[0,delta_t],args=(u,100))
            theta_omega0 = theta_omega[-1]

            v = odeint(self.model.ode_velocity,v0,[0,delta_t],args=(u,100))
            v0 = v[-1]

        return theta_store, omega_store, sp_store, v_store, u_store, ts, theta_noisy_store

    def plot(self, theta_store, omega_store, sp_store, v_store, u_store, ts):
        # plot results
        plt.rcParams["figure.figsize"] = (15,10)
        plt.plot(ts,theta_store,'b-',linewidth=3)
        plt.plot(ts,sp_store,'k--',linewidth=2)
        plt.ylabel('Theta (rad)')
        plt.show()