import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from time import time

class Quadrotor:
    
    def __init__(self, m=1, Ix=1, Iy=1, Iz=1, L=1, b=1, k=1, x0=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]):
        # Parameters for the system
        self.g = 0.1 # Gravity constant
        self.m = m # Mass of the quad-copter
        self.Ix = Ix # Moment of inertia about x-axis
        self.Iy = Iy # Moment of inertia about y-axis
        self.Iz = Iz # Moment of inertia about z-axis
        self.L = L # Length of the rotor arm
        self.b = b # Propeller drag coefficient
        self.k = k # Propeller lift coefficient
        self.state = x0 # [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        self.t = 0 # Time corresponding to initial conditions.
        
        
    def model(self, state, t):
      # Extract state variables
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
  
        # Define the control inputs
        F = -1000
        T1, T2, T3, T4 = [F, -F, F, -F] #u
        
        # Define the state-space model equations
        dxdt = vx
        dydt = vy
        dzdt = vz
        dvxdt = (T1 + T2 + T3 + T4)/self.m * np.cos(theta) * np.cos(psi)
        dvydt = (T1 + T2 + T3 + T4)/self.m * np.cos(theta) * np.sin(psi)
        dvzdt = (T1 + T2 + T3 + T4)/self.m * np.sin(theta) - self.g
        dphidt = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        dthetadt = q * np.cos(phi) - r * np.sin(phi)
        dpsidt = (q * np.sin(phi) + r * np.cos(phi))/np.cos(theta)
        dpdt = (self.Iy - self.Iz) / self.Ix * q * r + (T1 - T2 + T3 - T4) * self.k / self.Ix * self.L
        dqdt = (self.Iz - self.Ix) / self.Iy * p * r + (T1 - T2 - T3 + T4) * self.k / self.Iy * self.L
        drdt = (T1 + T2 - T3 - T4) * self.b / self.Iz
        
        # Return the state derivatives
        return [dxdt, dydt, dzdt, dvxdt, dvydt, dvzdt, dphidt, dthetadt, dpsidt, dpdt, dqdt, drdt]


    def step(self, dt):
        # Evaluate the system from previous state, one step further
        self.state = odeint(self.model, self.state, [0, dt])[1]
        self.t += dt
        return self.state
        

sys = Quadrotor(m=0.65, Ix=0.0075, Iy=0.0075, Iz=0.0075, L=0.23, b=0.1, k=0.0000313, x0=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
dt = 1.0/20 # 20 fps

# Initialize the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', xlim=(-5, 5), ylim=(-5, 5), zlim=(-1, 10))
line = ax.plot([], [], [], 'o-', lw=2)[0]
title = ax.text(x=0, y=0, z=14, s="")

def animate(frame_num, line, title, sys, dt):
    """perform animation step"""
    x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = sys.step(dt)
    line.set_data([x], [y])
    line.set_3d_properties([z], 'z')
    title.set_text(f"x: {x:0.2f}, y: {y:0.2f}, z: {z:0.2f}")
    return line, title,

ani = FuncAnimation(fig, animate, frames=100, interval=1000*dt, blit=True, repeat=False, fargs=(line, title, sys, dt))

plt.show(block=False)
plt.pause(10) # Close window after specified number of seconds