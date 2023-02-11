import numpy as np
from scipy.integrate import odeint, solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from time import time


class InvertedPendulumOnCart:

    def __init__(self, M=1.0, m=0.2, b=0.1, l=0.3, Jp=0.006, x0=[0, 0, 0, 0]):
        # Parameters for the system
        self.g = 9.81 # Gravity
        self.M = M  # Mass of the cart (kg)
        self.m = m  # Mass of the pendulum (kg)
        self.b = b  # Coefficient of friction for cart (Ns/m=(N/m)/s)
        self.l = l  # Length of the pendulum (m)
        self.Jp = Jp  # Mass moment of inertia of the pendulum (kg.m^2)

        # Position, Velocity of Cart, Angle and Angular Velocity of Pendulum
        self.state = x0  # [position, velocity, angle, angular velocity]
        self.old_state = x0 # Required by the model
        self.t = 0  # Time corresponding to initial conditions.

    def model(self, state, t, input):
      # Extract state variables
        x, v, theta, omega = state
        _, dv, _, domega = self.old_state
        Force = input
        k1 = 1.0 / (self.M + self.m)
        k2 = 1.0 / (self.Jp + self.m*self.l**2)
        
        dx_dt = v
        dv_dt = -(k1* self.b)*v - (k1*self.m*self.l*np.cos(theta))*domega - (k1*self.m*self.l*np.sin(theta))*omega**2 + (k1 * Force)
        dtheta_dt = omega
        domega_dt = -(k2*self.m*self.l*np.cos(theta)) * dv  + (k2*self.m*self.g*self.l*np.sin(theta))

        dstate_dt = [dx_dt, dv_dt, dtheta_dt, domega_dt]
        self.old_state = dstate_dt
        
        return dstate_dt

    def step(self, dt, input):
        # Evaluate the system from previous state, one step further
        self.state=odeint(self.model, self.state, [0, dt], args=(input,))[1]
        self.t += dt
        return self.state


sys = InvertedPendulumOnCart(M=1.0, m=0.2, b=0.1, l=0.3,Jp=0.006, x0=[0, 0, 0, 0])
dt=1.0/20  # 20 fps

# Initialize the figure and 3D axis
fig=plt.figure()
ax=fig.add_subplot(111, aspect='equal', autoscale_on=True,
                   xlim=(0, 10), ylim=(-3, 3))
line=ax.plot([], [], 'o-', lw=2)[0]
title=ax.text(x=0, y=0, s="")


def animate(frame_num, line, title, sys, dt):
    """perform animation step"""
    force=0
    if frame_num < (0.5/dt):
        force=1
    elif frame_num < (1/dt):
        force=-1
    pos, vel, theta, omega=sys.step(dt, force)
    line.set_data([sys.t], [pos])
    title.set_text(
        f"t: {sys.t:0.2f}, position: {pos:0.2f}, velocity: {vel:0.2f}, theta: {theta:0.2f}, omega: {omega:0.2f}, force: {force:0.2f}")
    return line, title,


ani=FuncAnimation(fig, animate, frames=10000, interval=1000*dt,
                    blit=True, repeat=False, fargs=(line, title, sys, dt))

plt.show(block=False)
plt.pause(10)  # Close window after specified number of seconds
