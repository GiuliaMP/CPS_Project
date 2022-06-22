import numpy as np
import car_model
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def run(tf, nsteps, sp, theta_omega0, noise, Kp, Ki, Kd):

    # tf: final time for simulation
    # nsteps: number of time steps
    # sp: vector of the reference angle (length == nsteps )
    # theta_omega0: initial condition for theta and omega [theta, omega]
    # noise: boolean representing if we want to use noise or not
    # Kp, Ki, Kd: PID parameters

    # Define the constant and initialize the model
    eta = 0.25 # damping factor
    m_b = 0.01 # mass of the ball
    m_c = 500 # mass of the car
    g = 9.8 # gravity
    r = 1 # radius of the pendulum
    model = car_model.Model(eta, m_b,m_c,g,r)

    # Define the timesteps
    delta_t = tf/(nsteps)   # length of each timestep
    ts = np.linspace(0,tf,nsteps)

    # utility list for storing
    sp_store = np.zeros(nsteps)
    omega_store = np.zeros(nsteps)
    theta_store = np.zeros(nsteps)
    u_store = np.zeros(nsteps)

    # PID utilities
    sum_int = 0.0
    error = 0
    u = 0

    # store the speed to perform later some monitoring
    v0 = 0
    v_store = np.zeros(nsteps)

    # simulate with ODEINT
    for i in range(nsteps):
        
        u_store[i] = u
        v_store[i] = v0
        sp_store[i] = sp[i]
        omega_store[i] = theta_omega0[1] 
        theta_store[i] = theta_omega0[0]

        # I want to save the clear data but I want to compute the new u having some noise in the observations
        if noise:
            theta_omega0[0] =theta_omega0[0] + np.random.normal(0.0,0.1, 1) * 10**(-1)
        
        prec_error = error
        error = sp[i] - theta_omega0[0]
        sum_int += error*delta_t
        
        #if i > 0:
            #numerator_D = (theta_omega0[0]-theta_store[i-1]) # with observed for the derivative kick
        numerator_D = (error-prec_error) # with error
        #u = Kp*error + Ki * sum_int # PI controller (10,50)
        #u = Kp*error - Kd * (numerator_D/(delta_t)) # PD controller
        u = Kp*error + Ki * sum_int - Kd * (numerator_D/(delta_t)) # PID controller # (10,50,30)

        # Bounds for the controlled variable
        if u >= 100.0:
            u = 100.0
            sum_int = sum_int - error*delta_t
        if u <= -50.0:
            u = -50.0
            sum_int = sum_int - error*delta_t
        
        theta_omega = odeint(model.ode_angle,theta_omega0,[0,delta_t],args=(u,100))
        theta_omega0 = theta_omega[-1]

        v = odeint(model.ode_velocity,v0,[0,delta_t],args=(u,100))
        v0 = v[-1]

    return theta_store, omega_store, sp_store, v_store, u_store, ts

def plot(theta_store, omega_store, sp_store, v_store, u_store, ts):
    # plot results
    plt.rcParams["figure.figsize"] = (15,10)
    #plt.subplot(2,1,1)
    plt.plot(ts,theta_store,'b-',linewidth=3)
    plt.plot(ts,sp_store,'k--',linewidth=2)
    plt.ylabel('Theta (rad)')
    #plt.legend(['Theta','Set Point'],loc=2)
    '''plt.subplot(2,1,2)
    plt.plot(ts,v_store,'r--',linewidth=3)
    plt.ylabel('u')    
    plt.legend(['Gas Pedal (%)'])
    plt.xlabel('Time (sec)')'''
    plt.show()
