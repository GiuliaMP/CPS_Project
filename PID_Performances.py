from cmath import inf
import numpy as np


def rise_time(x, ref, time):
    """Time point at which the output signal crosses the desired reference value"""
    i = 0
    for x_val in x:
        if (x_val + 0.001) >= ref:
            return time[i]
        i+=1
    return inf

def overshoot(x, ref):
    """Difference between the max value of the system output and the desired reference value"""
    if max(x) >= ref:
        return round(max(x)-ref, 3)
    return 0 

def steady_state_error(x, ref):
    """Difference between steady state value of the output signal and value of the reference signal"""
    return (x[-1] - ref)


def settling_time(x, ref, time):
    """Time at which the output reaches its steady state value"""
    i = 0
    while x[i] > x[-1] + 0.0001 or x[i] < x[-1] - 0.0001:
        i+=1
    return time[i]