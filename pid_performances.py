from cmath import inf
import numpy as np

def rise_time(x, ref, time):
    """Time point at which the output signal crosses the desired reference value"""
    i = 0
    for x_val in x:
        if (x_val >= ref[i]):
            return round(time[i],3)
        i+=1
    return inf

def overshoot(x, ref):
    """Difference between the max value of the system output and the desired reference value"""
    ind = np.where(x == np.max(x))
    if x[ind][0] >= ref[ind][0]:
        return round(x[ind][0]-ref[ind][0], 3)
    return 0 

def steady_state_error(x, ref):
    """Difference between steady state value of the output signal and value of the reference signal"""
    return round(np.abs(x[-1] - ref[-1]),3)


def settling_time(x, time):
    """Time at which the output reaches its steady state value"""
    i = 0
    while x[i] > x[-1] + 0.001 or x[i] < x[-1] - 0.001:
        i+=1
    return round(time[i],3)


def print_performances(theta_store, sp_store, ts):
    print(f'overshoot: {overshoot(theta_store, sp_store)}')
    print(f'rise time: {rise_time(theta_store, sp_store, ts)}')
    print(f'steady state error: {steady_state_error(theta_store, sp_store)}')
    print(f'settling time: {settling_time(theta_store, ts)}')