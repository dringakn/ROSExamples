#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    This script defines and animates a planar quadrotor (2D) dynamics model,
    including thrust and moment control, state integration, and real-time plotting.

Features:
  • Rigid‐body dynamics:
      – 6‐state model [y, z, φ, ẏ, ż, φ̇]
      – Gravity, rotor thrust limits, moment of inertia
  • Control:
      – PD controller for vertical (z), lateral (y), and attitude (φ)
      – Computes total thrust F and moment M to track desired trajectories
  • Simulation:
      – Integrates equations of motion via SciPy’s `odeint`
      – Timestep configurable (default 1/20 s for 20 Hz update)
  • Visualization:
      – 2D real‐time animation of (y, z) position with Matplotlib FuncAnimation
      – Displays current time, position, attitude (ψ), and velocities
  • Usage:
      – Simply run: `python3 planar_quadrotor.py`
      – Edit `desired` trajectory or controller gains as needed
  • Dependencies:
      – numpy, scipy, matplotlib
      – mpl_toolkits.mplot3d (for future 3D extensions)

"""

import numpy as np
from scipy.integrate import odeint, solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from time import time


class PlanarQuadrotor:

    def __init__(self, m=0.18, Ixx=0.00025, l=0.086, x0=[0, 0, 0, 0, 0, 0]):
        # Initialize states
        self.t = 0  # Time corresponding to initial conditions.
        self.state = x0  # [y, z, phi, y', z', phi']
        # Parameters for the system
        self.g = 9.81  # Gravity constant
        self.m = m  # Mass of the quad-copter
        self.Ixx = Ixx  # Moment of inertia about x-axis
        self.l = l  # Length of the rotor arm
        self.Fmin = 0
        self.Fmax = 2 * self.m * self.g # Maximum Quadrotor thrust
        
    def model(self, state, t, input):
      # Extract state variables
        y, z, phi, dy, dz, dphi = state
        F, M = input 
        
        #     u1      u2
        #   _____    _____
        #     |________|
        # 
        #  F = u1 + u2;
        #  M = (u2 - u1)*l
        #  Check maximum force possible by each rotor
        u1 = (F - M/self.l) / 2.0
        u2 = (F + M/self.l) / 2.0
        u1 = min(max(u1, self.Fmin/2), self.Fmax/2)  # Clamp maximum force by first rotor 
        u2 = min(max(u2, self.Fmin/2), self.Fmax/2)  # Clamp maximum force by second rotor 
        F = u1 + u2
        M = (u2 - u1) * self.l
        
        dy_dt = dy
        dz_dt = dz
        dphi_dt = dphi
        d2y_dt = -F * np.sin(phi) / self.m
        d2z_dt = -self.g + F * np.cos(phi) / self.m
        d2phi_dt = M /self.Ixx

        dstate_dt = [dy_dt, dz_dt, dphi_dt, d2y_dt, d2z_dt, d2phi_dt]
        
        return dstate_dt

    def step(self, dt, input):
        # Evaluate the system from previous state, one step further
        self.state = odeint(self.model, self.state, [0, dt], args=(input,))[1]
        self.t += dt
        return self.state

    def controller(self, desired):
        # kpy, kdy = 40.01, 10.01 
        # kpz, kdz = 800.01, 10.01 
        # kpphi, kdphi = 1600.01, 25.01 
        kpz, kdz = 20.01, 20.01 
        kpy, kdy = 20.01, 5.01 
        kpphi, kdphi = 1000.01, 10.01 

        y, z, phi, vy, vz, dphi = self.state
        y_r, z_r, vy_r, vz_r, ay_r, az_r = desired
        
        y_err = y_r - y
        vy_err = vy_r - vy
        
        z_err = z_r - z
        vz_err = vz_r - vz
        
        # Angle error can be calculated as follows
        phi_c = -(1.0/self.g) * (ay_r + (kpy * y_err) + (kdy * vy_err))
        phi_err = phi_c - phi
        dphi_err = 0 - dphi
        
        M = self.Ixx * (0 + (kpphi * phi_err) + (kdphi * dphi_err))
        F = self.m * (self.g + az_r + (kpz * z_err) + (kdz * vz_err))
        
        if F < self.Fmin:
            F = self.Fmin
        if F > self.Fmax:
            F = self.Fmax
            
        return F, M
    
sys = PlanarQuadrotor(x0=[0,0,0,0,0,0])
dt = 1.0/20  # 20 fps

# Initialize the figure and 3D axis
fig = plt.figure(figsize=(16, 16))
ax = fig.add_subplot(111, aspect='equal', autoscale_on=True, xlim=(-10, 10), ylim=(-10, 10))
line = ax.plot([], [], 'o-', lw=2)[0]
title = ax.text(x=-9, y=0, s="")


def animate(frame_num, line, title, sys, dt):
    """perform animation step"""
    # desired: x_des, y_des, vx_des, vy_des, ax_des, ay_des
    control_inputs = sys.controller(desired=[0, 5, 0, 0, 0, 0])
    x, y, psi, vx, vy, omega = sys.step(dt, control_inputs)
    line.set_data([x], [y])
    title.set_text(f"t: {sys.t:0.2f}, x: {x:0.2f}, y: {y:0.2f}, psi: {np.rad2deg(psi):0.2f}, vx: {vx:0.2f}, vy: {vy:0.2f}")
    return line, title,


ani = FuncAnimation(fig, animate, frames=10000, interval=1000*dt,
                    blit=True, repeat=False, fargs=(line, title, sys, dt))

plt.grid('both')
plt.show(block=False)
plt.pause(2*60)  # Close window after specified number of seconds
