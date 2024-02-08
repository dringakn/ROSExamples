#! /bin/python3
import numpy as np
import matplotlib.pyplot as plt

# Given function


def computeTimeVelocityRamp(start, goal, v_max, a_max, v_initial, v_final):
    distance = np.linalg.norm(start - goal)
    acc_time = (v_max - v_initial) / a_max
    dec_time = (v_max - v_final) / a_max
    acc_distance = 0.5 * (v_initial + v_max) * acc_time
    dec_distance = 0.5 * (v_final + v_max) * dec_time

    if distance < acc_distance + dec_distance:
        return acc_time + (distance - acc_distance) / v_max
    else:
        return acc_time + dec_time + (distance - acc_distance - dec_distance) / v_max


# Parameters
start = np.array([0.0, 0.0])
goal = np.array([10.0, 10.0])
v_initial = 0.0
v_max = 1.0
v_final = 0.0
a_max = 1.0

# Calculate time
time = computeTimeVelocityRamp(start, goal, v_max, a_max, v_initial, v_final)

# Plotting the motion profile
t_acc = (v_max - v_initial) / a_max
t_dec = (v_max - v_final) / a_max
t_total = time
t = np.linspace(0, t_total, 100)
v = np.piecewise(t, [t <= t_acc, (t > t_acc) & (t <= t_total - t_dec), t > t_total - t_dec],
                 [lambda t: v_initial + a_max * t, v_max, lambda t: v_final + a_max * (t_total - t)])

plt.plot(t, v)
plt.xlabel("Time")
plt.ylabel("Velocity")
plt.title(f"Velocity Profile, Time required: {time:0.3f}")
plt.grid(True)
plt.show()
