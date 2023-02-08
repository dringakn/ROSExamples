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