#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    Simulates and animates a spring–mass–damper system using numerical integration
    of its ODEs and Matplotlib for real‐time visualization.

Features:
  • Configurable system parameters:
      – m: mass (kg)
      – k: spring constant (N/m)
      – c: damping coefficient (N·s/m)
      – x0: initial state [position (m), velocity (m/s)]
  • ODE integration via scipy.integrate.odeint
  • Step‑by‑step simulation interface (`step()`) with adjustable time step
  • Reset capability to restart simulation from arbitrary initial conditions
  • Real‑time animation using Matplotlib’s FuncAnimation
  • Adjustable frame rate (frames per second) and total frames
  • Self‑contained example at bottom showing 20 fps animation for 100 frames

Usage:
    Simply run this script:
        $ python spring_mass_damper.py
    Modify the `SpringMassDamper(...)` parameters, `dt`, and animation settings
    as desired.

Requirements:
    • numpy
    • scipy
    • matplotlib
"""
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from time import time

class SpringMassDamper:
    def __init__(self, m=1, k=10, c=1, x0=[0, 1]):
        # Parameters for the system
        self.m = m # mass (kg)
        self.k = k # spring constant (N/m)
        self.c = c # damping constant (N·s/m)
        self.state = x0 # Initial conditions for the state variables [x, v]
        self.t = 0 # Time corresponding to initial conditions.
        
    def model(self, x, t):
        # Define the state-space equations for a spring-mass-damper system
        x1, x2 = x
        dx1dt = x2
        dx2dt = -(self.k/self.m)*x1 - (self.c/self.m)*x2
        return [dx1dt, dx2dt]
    
    def step(self, dt):
        # Evaluate the system from previous state, one step further
        self.state = odeint(self.model, self.state, [0, dt])[1]
        self.t += dt
        return self.t, self.state[0]

    def reset(self, t0=0, x0=[0, 1]):
        self.t = t0 # Time corresponding to initial conditions.
        self.state = x0 # Initial conditions for the state variables [x, v]
        

sys = SpringMassDamper(m=1, k=10, c=1, x0=[0, 1])
dt = 1.0/20 # 20 fps

fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=True, xlim=(-0.5, 6), ylim=(-2, 2))
line, = ax.plot([], [], 'o-', lw=2)

def animate(frame_num):
    """perform animation step"""
    line.set_data(*sys.step(dt))
    return line, 

ani = FuncAnimation(fig, animate, frames=100, interval=1000*dt, blit=True, repeat=False)

plt.show(block=False)
plt.pause(10) # Close window after 10 seconds