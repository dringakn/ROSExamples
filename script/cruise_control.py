#!/usr/bin/env python3
"""
Author:    Dr. Ing. Ahmad Kamal Nasir
Email:     dringakn@gmail.com

Description:
    Simulates and animates a simple cruise control system (mass-damper) driven by a piecewise-constant force.

Features:
  • Second-order ODE model: M·dv/dt + B·v = Force  
  • Numerical integration via scipy.integrate.odeint  
  • Real-time animation of position vs. time with matplotlib FuncAnimation  
  • Force profile: +100 N for t∈[0,2 s), –100 N for t∈[2,4 s), then 0 N  
  • Configurable mass (M), damping (B), and initial state  

Dependencies:
  • numpy  
  • scipy (integrate.odeint)  
  • matplotlib  

Usage:
    python3 cruise_control.py
"""

import numpy as np
from scipy.integrate import odeint, solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from time import time


class CruiseControl:

    def __init__(self, M=1000, B=50, x0=[0, 0]):
        # Parameters for the system
        self.M = M  # Mass of the vehicle (kg)
        self.B = B  # Damping constant(Ns/m)
        
        self.state = x0  # [position, velocity]
        self.t = 0  # Time corresponding to initial conditions.

    def model(self, state, t, input):
      # Extract state variables
        x, v = state
        Force = input
        dx_dt = v
        dv_dt = -(self.B/self.M) * v + (Force/self.M)

        dstate_dt = [dx_dt, dv_dt]
        
        return dstate_dt

    def step(self, dt, input):
        # Evaluate the system from previous state, one step further
        self.state = odeint(self.model, self.state, [0, dt], args=(input,))[1]
        self.t += dt
        return self.state


sys = CruiseControl(M=1000, B=50, x0=[0, 0])
dt = 1.0/20  # 20 fps

# Initialize the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=True, xlim=(0, 10), ylim=(-3, 3))
line = ax.plot([], [], 'o-', lw=2)[0]
title = ax.text(x=0, y=0, s="")


def animate(frame_num, line, title, sys, dt):
    """perform animation step"""
    force = 0
    if frame_num < (2/dt):
        force = 100
    elif frame_num < (4/dt):
        force = -100
    pos, vel = sys.step(dt, force)
    line.set_data([sys.t], [pos])
    title.set_text(f"t: {sys.t:0.2f}, position: {pos:0.2f}, velocity: {vel:0.2f}, force: {force:0.2f}")
    return line, title,


ani = FuncAnimation(fig, animate, frames=10000, interval=1000*dt,
                    blit=True, repeat=False, fargs=(line, title, sys, dt))

plt.show(block=False)
plt.pause(10)  # Close window after specified number of seconds
