#!/usr/bin/env python3
"""
Author:    Dr. Ing. Ahmad Kamal Nasir
Email:     dringakn@gmail.com

Description:
    Simulates and animates a DC motor’s rotational dynamics
    under PID voltage control, plotting angular velocity over time.

Features:
  • DCMotor class modeling mechanical (J, B) and electrical (R, L, Ke, Kt) dynamics  
  • State integration via scipy.integrate.odeint  
  • PIDController for closed-loop voltage input  
  • Real-time 2D plot (t vs. ω) using Matplotlib FuncAnimation  
  • Easy parameter tweaking for motor and controller  

Dependencies:
  – numpy  
  – scipy  
  – matplotlib  
  – pid_controller.PIDController  

Usage:
    python3 dc_motor_sim.py
"""

import numpy as np
from scipy.integrate import odeint, solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from time import time
from pid_controller import PIDController

class DCMotor:

    def __init__(self, J=0.01, B=0.1, Kt=0.01, Ke=0.01, R=1, L=0.5, x0=[0, 0, 0]):
        # Parameters for the system
        self.J = J  # Moment of inertia (kg*m^2)
        self.B = B  # Motor viscous friction constant(Nms)
        self.Kt = Kt  # Torque constant (Nm/A)
        self.Ke = Ke  # EMF constant (V/(rad/s))
        self.R = R  # Armature resistance (Ω)
        self.L = L  # Armature inductance (H)
        
        self.state = x0  # [omega, current]
        self.t = 0  # Time corresponding to initial conditions.

    def model(self, state, t, input):
      # Extract state variables
        theta, omega, current = state
        voltage = input
        dtheta_dt = omega
        domega_dt = (self.Kt/self.J) * current - (self.B/self.J) * omega
        dcurrent_dt = -(self.R/self.L) * current - (self.Ke/self.L) * omega + voltage/self.L

        dstate_dt = [dtheta_dt, domega_dt, dcurrent_dt]
        
        return dstate_dt

    def step(self, dt, input):
        # Evaluate the system from previous state, one step further
        self.state = odeint(self.model, self.state, [0, dt], args=(input,))[1]
        self.t += dt
        return self.state


sys = DCMotor(J=0.005, B=0.1, Kt=0.01, Ke=0.01, R=1, L=0.5, x0=[0, 0, 0])
pid = PIDController(1, 100, 200, 1)
dt = 1.0/20  # 20 fps

# Initialize the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=True, xlim=(0, 15), ylim=(-1.5, 1.5))
line = ax.plot([], [], 'o-', lw=2)[0]
title = ax.text(x=0, y=0, s="")
old_omega = 0

def animate(frame_num, line, title, sys, dt):
    """perform animation step"""
    global old_omega
    voltage = pid.calculate_input(old_omega, dt)
    theta, omega, current = sys.step(dt, voltage)
    old_omega = omega
    line.set_data([sys.t], [omega])
    title.set_text(f"t: {sys.t:0.2f}, theta: {theta:0.2f}, omega: {omega:0.2f}, current: {current:0.2f}, voltage: {voltage:0.2f}")
    return line, title,


ani = FuncAnimation(fig, animate, frames=60000, interval=1000*dt,
                    blit=True, repeat=False, fargs=(line, title, sys, dt))

plt.show(block=False)
plt.pause(60)  # Close window after specified number of seconds
