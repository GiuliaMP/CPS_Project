import numpy as np
import car_model
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def run(tf, nsteps, sp, theta_omega0):

    # Run with noise
    noise = False

    # Define the model
    eta = 0.25
    m_b = 0.01 # mass of the ball
    m_c = 500 # mass of the car
    g = 9.8 # gravity
    r = 1 # radius of the pendulum
    model = car_model.Model(eta, m_b,m_c,g,r)

    # Define T_Horizon and timesteps
    #tf = 300           # final time for simulation
    #nsteps = 300             # number of time steps
    delta_t = tf/(nsteps)   # how long is each time step?
    ts = np.linspace(0,tf,nsteps) # linearly spaced time vector

    # velocity initial condition
    #theta_omega0 = [0.0,0.0]

    # utility list for storing
    sp_store = np.zeros(nsteps)
    omega_store = np.zeros(nsteps)
    theta_store = np.zeros(nsteps)
    step = np.zeros(nsteps)

    # PID values
    ubias = 0.0
    sum_int = 0.0
    Kp = 15#Kc
    Ki = 30#Kc/tauI
    Kd = 30#Kc*tauD
    # (10,50,30)

    v_store = np.zeros(nsteps)
    v0 = 0

    error = 0
    # simulate with ODEINT
    for i in range(nsteps):
        #sp = np.sin(i/20)/10 + 0.2
        '''if i == 50:
            sp = 0.2
        if i == 100:
            sp = 0.0
        if i == 150:
            sp = 0.78'''
        #sp = 0.78
        
            
        sp_store[i] = sp[i]
        omega_store[i] = theta_omega0[1] # store the velocity and the angle for plotting
        theta_store[i] = theta_omega0[0] # store the velocity and the angle for plotting
        v_store[i] = v0
        
        prec_error = error
        error = sp[i] - theta_omega0[0]
        sum_int += error*delta_t
        numerator_D = 0.0
        if i > 0:
            #numerator_D = (theta_omega0[0]-theta_store[i-1]) # with observed for the derivative kick
            numerator_D = (error-prec_error) # with error
        #u = ubias + Kp*error + Ki * sum_int # PI controller (10,50)
        #u = ubias + Kp*error - Kd * (numerator_D/(delta_t)) # PD controller (non funge bene perché ....)
        u = ubias + Kp*error + Ki * sum_int - Kd * (numerator_D/(delta_t)) # PID controller # (10,50,30)
        #u = 50
        if u >= 100.0:
            u = 100.0
            sum_int = sum_int - error*delta_t
        if u <= -50.0:
            u = -50.0
            sum_int = sum_int - error*delta_t
        step[i] = u
        
        theta_omega = odeint(model.ode_angle,theta_omega0,[0,delta_t],args=(u,100))
        if noise:
            theta_omega0 = theta_omega[-1] + np.random(0.0, 0.1, 1) * 10**(-2)    # take the last value
        else:
            theta_omega0 = theta_omega[-1]

        v = odeint(model.ode_velocity,v0,[0,delta_t],args=(u,100))
        v0 = v[-1]

    return theta_store, omega_store, sp_store, v_store, step, ts

def plot(theta_store, omega_store, sp_store, v_store, step, ts):
    # plot results
    plt.rcParams["figure.figsize"] = (15,10)
    plt.subplot(2,1,1)
    plt.plot(ts,theta_store,'b-',linewidth=3)
    plt.plot(ts,sp_store,'k--',linewidth=2)
    plt.ylabel('Theta (rad)')
    #plt.legend(['Theta','Set Point'],loc=2)
    plt.subplot(2,1,2)
    plt.plot(ts,v_store,'r--',linewidth=3)
    plt.ylabel('u')    
    plt.legend(['Gas Pedal (%)'])
    plt.xlabel('Time (sec)')
    plt.show()
