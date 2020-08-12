import numpy as np


def mean(values):
    return np.mean(values)

def stdev(values):
    return np.std(values)

def outlier_count(values, thresholds):
    count = 0
    for v in values:
        if (v<thresholds[0] or v>thresholds[1]):
            count += 1
        else:
            continue
    return count

def uptime(timestamps, threshold):
    t0 = min(timestamps)
    t_prev = t0
    timeout_acc = 0
    for t in timestamps[1:]:
        dt = t-t_prev
        t_prev = t
        if dt>threshold:
            timeout_acc += dt
        else:
            continue
    
    t_tot = t-t0
    uptime = 100*(t_tot-timeout_acc)/t_tot
    return uptime