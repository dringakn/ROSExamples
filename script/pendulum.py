import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from time import time

class Pendulum:
    def __init__(self, m=1, l=1, u=0, x0=[0.0, 0.0, np.pi/4, 0.0]):
        # Parameters for the system
        self.m = m  # mass of pendulum
        self.l = l  # length of pendulum
        self.g = 9.81  # acceleration due to gravity
        self.u = u  # control input
        self.state = x0 # Initial conditions for the state variables [x, v, theta, omega]
        self.t = 0 # Time corresponding to initial conditions.
        
    def model(self, state, t):
        # Define the state-space equations for a spring-mass-damper system
        x, x_dot, theta, theta_dot = state
        dx_dt = x_dot
        dxdot_dt = (self.u - self.m * self.g * self.l * np.sin(theta)) / (self.m * self.l ** 2)
        dtheta_dt = theta_dot
        dthetadot_dt = (self.g * np.sin(theta) - self.u * np.cos(theta)) / (self.l)
        return [dx_dt, dxdot_dt, dtheta_dt, dthetadot_dt]
    
    def step(self, dt):
        # Evaluate the system from previous state, one step further
        self.state = odeint(self.model, self.state, [0, dt])[1]
        self.t += dt
        return self.state[0], self.state[1]

    def reset(self, t0=0, x0=[0.0, 0.0, np.pi/4, 0.0]):
        self.t = t0 # Time corresponding to initial conditions.
        self.state = x0 # Initial conditions for the state variables [x, v]
        

sys = Pendulum(m=1, l=10, u=1, x0=[0.0, 0.0, np.pi/4, 0.0])
dt = 1.0/20 # 20 fps

fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=True, xlim=(-7, 1), ylim=(-2, 2))
line, = ax.plot([], [], 'o-', lw=2)

def animate(frame_num):
    """perform animation step"""
    line.set_data(*sys.step(dt))
    return line, 

ani = FuncAnimation(fig, animate, frames=1000, interval=1000*dt, blit=True, repeat=False)

plt.show(block=False)
plt.pause(30) # Close window after specified number of seconds