import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from time import time

class Quadrotor:
    
    def __init__(self, m=0.65, Ix=0.0075, Iy=0.0075, Iz=0.013, Ir=6e-5, L=0.23, Kf=3.13e-5, Km=7.5e-7, 
                 Kt=0.1, Kr=0.1, x0=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]):
        # Parameters for the system
        self.g = 9.81 # Gravity constant (m/s^2)
        self.m = m # Mass of the quad-copter (kg)
        self.Ix = Ix # Moment of inertia about x-axis (kg.m^2)
        self.Iy = Iy # Moment of inertia about y-axis (kg.m^2)
        self.Iz = Iz # Moment of inertia about z-axis (kg.m^2)
        self.Ir = Ir # Rotor moment of inertia rotationa axis (kg.m^2)
        self.L = L # Length of the rotor arm (m)
        self.Kf = Kf # Thrust coefficient (N.s^2)
        self.Km = Km # Moment coefficient (Nm.s^2)
        self.Kt = Kt # Aerodynamic thrust drag coefficient (Ns/m)
        self.Kr = Kr # Aerodynamic moment drag coefficient (Nm.s)
        
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


    def model1(self, state, t, inputs):
      # Extract state variables
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
  
        # Define the control inputs, rotational speed of four rotors
        o1, o2, o3, o4 = inputs
        u1 = self.Kf * (o1**2 + o2**2 + o3**2 + o4**2) # Thrust
        u2 = self.Kf * (o4**2 - o2**2) # Roll
        u3 = self.Kf * (o1**2 - o3**2) # Pitch
        u4 = self.Km * (o1**2 - o2**2 + o3**2 - o4**2) # Yaw
        omega_r = o1 - o2 + o3 - o4
                
        # Define the state-space model equations
        dxdt = vx
        dydt = vy
        dzdt = vz
        dvxdt = -1/self.m * (self.Kt * vx + u1 * (np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.sin(theta)))
        dvydt = -1/self.m * (self.Kt * vy + u1 * (np.sin(phi) * np.cos(psi) - np.cos(phi) * np.sin(psi) * np.sin(theta)))
        dvzdt = -1/self.m * (self.Kt * vz + u1 * np.cos(phi) * np.cos(theta) - self.m * self.g)
        dphidt = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        dthetadt = q * np.cos(phi) - r * np.sin(phi)
        dpsidt = (q * np.sin(phi) + r * np.cos(phi))/np.cos(theta)
        dpdt = -1/self.Ix * (q * r * (self.Iz - self.Iy) - self.L * u2 + self.Kr * p + self.Ir * q * omega_r)
        dqdt = -1/self.Iy * (p * r * (self.Iz - self.Ix) + self.L * u3 - self.Kr * q + self.Ir * p * omega_r)
        drdt = -1/self.Iz * (u4 + p * q * (self.Ix - self.Iy) - self.Kr * r)
        
        # Return the state derivatives
        return [dxdt, dydt, dzdt, dvxdt, dvydt, dvzdt, dphidt, dthetadt, dpsidt, dpdt, dqdt, drdt]


    def model2(self, state, t, inputs):
      # Extract state variables
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
  
        # Define the control inputs
        # u1: Thrust, F1+F2+F3+F4
        # u2: Roll, L(F2 - F4)
        # u3: Pitch, L(F3 - F1)
        # u4: Yaw, M1-M2+M3-M4
        u1, u2, u3, u4 = inputs
                
        # Define the state-space model equations
        dxdt = vx
        dydt = vy
        dzdt = vz
        dvxdt = 1/self.m * (u1 * (np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi) * np.cos(theta))) # u₁⋅(sin(φ(t))⋅sin(ψ(t))⋅cos(θ(t)) + sin(θ(t))⋅cos(ψ(t)))/m
        dvydt = 1/self.m * (u1 * (np.sin(psi) * np.sin(theta) - np.sin(phi) * np.cos(psi) * np.cos(theta))) # u₁⋅(-sin(φ(t))⋅cos(ψ(t))⋅cos(θ(t)) + sin(ψ(t))⋅sin(θ(t)))/m
        dvzdt = 1/self.m * (u1 * np.cos(phi) * np.cos(theta) - self.m * self.g) # u₁⋅cos(φ(t))⋅cos(θ(t))/m - g
        dphidt = p * np.cos(theta) + r * np.sin(theta) # p(t)⋅cos(θ(t)) + r(t)⋅sin(θ(t))
        dthetadt = q + (p * np.sin(theta) - r * np.cos(theta)) * np.tan(phi) # p(t)⋅sin(θ(t))⋅tan(φ(t)) + q(t) - r(t)⋅cos(θ(t))⋅tan(φ(t))
        dpsidt = (-p * np.sin(theta) + r * np.cos(theta)) / np.cos(phi) # (-p(t)⋅sin(θ(t)) + r(t)⋅cos(θ(t))) / cos(φ(t))
        dpdt = 1/self.Ix * (u2 + q * r * (self.Iy - self.Iz)) # (Iyy⋅q(t)⋅r(t) - Izz⋅q(t)⋅r(t) + u₂)/Ixx
        dqdt = 1/self.Iy * (u3 + p * r * (self.Iz - self.Ix)) # (-Ixx⋅p(t)⋅r(t) + Izz⋅p(t)⋅r(t) + u₃)/Iyy
        drdt = 1/self.Iz * (u4 + p * q * (self.Ix - self.Iy)) # (Ixx⋅p(t)⋅q(t) - Iyy⋅p(t)⋅q(t) + u₄)/Izz
        
        # Return the state derivatives
        return [dxdt, dydt, dzdt, dvxdt, dvydt, dvzdt, dphidt, dthetadt, dpsidt, dpdt, dqdt, drdt]


    def controller(self, desired):
        Kp_tra = np.array([1, 1, 10])
        Kd_tra = np.array([2, 2, 10])
        Kp_ang = np.array([100, 100, 100])
        Kd_ang = np.array([1, 1, 1])

        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = self.state
        x_r, y_r, z_r, vx_r, vy_r, vz_r, ax_r, ay_r, az_r, psi_r, dpsi_r = desired
        
        pos_err = np.array([x_r - x, y_r - y, z_r - z])
        lin_vel_err = np.array([vx_r - vx, vy_r - vy, vz_r - vz])
        des_acc = np.array([ax_r, ay_r, az_r])
        
        cmd_acc = des_acc + Kp_tra * pos_err + Kd_tra * lin_vel_err # F, Force vector
        u1 = self.m * (self.g + cmd_acc[2]) # u1: Thrust force along z-axis, gravity compensation
        
        # Angle error can be calculated as follows
        phi_r = (1.0/self.g) * (cmd_acc[0] * np.sin(psi_r) - cmd_acc[1] * np.cos(psi_r))
        theta_r = (1.0/self.g) * (cmd_acc[0] * np.cos(psi_r) + cmd_acc[1] * np.sin(psi_r))
        
        angle_err = np.array([phi_r - phi, theta_r - theta, psi_r - psi])
        ang_vel_err = np.array([0 - p, 0 - q, dpsi_r - r])
        
        cmd_tor = Kp_ang * angle_err + Kd_ang * ang_vel_err # M, Torque vector
        u2 = cmd_tor[0] # u2: Roll torque
        u3 = cmd_tor[1] # u2: Pitch torque
        u4 = cmd_tor[2] # u2: Yaw torque
        
        return [u1, u2, u3, u4]
    

    def step(self, dt, input):
        # Evaluate the system from previous state, one step further
        self.state = odeint(self.model2, self.state, [0, dt], args=(input,))[1]
        self.t += dt
        return self.state
        

sys = Quadrotor( x0=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] )
dt = 1.0/100 # 100 fps

# Initialize the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', xlim=(-10, 10), ylim=(-10, 10), zlim=(-1, 12))
ax.set_xlabel("X[m]")
ax.set_ylabel("Y[m]")
ax.set_zlabel("Z[m]")
line = ax.plot([], [], [], 'o-', lw=2)[0]
arrow = ax.quiver(0, 0, 0, 1, 0, 0, length=1) # Fix
title = ax.text(x=-18, y=0, z=10, s="")

def animate(frame_num, line, title, sys, dt):
    """perform animation step"""
    input = sys.controller(desired=[5,5,5, 0,0,0, 0,0,0, 0,0]) # [x,y,z], [vx,vy,vz], [ax,ay,az], psi, dpsi
    x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = sys.step(dt, input)
    line.set_data([x], [y])
    line.set_3d_properties([z], 'z')
    # arrow.set_UVC(x, y, z)
    # arrow.set_segments([[x, y, z], [x + 1, y + 1, z + 1]])
    title.set_text(f"x:{x:0.2f} y:{y:0.2f} z:{z:0.2f}\nvx:{vx:0.2f} vy:{vy:0.2f} vz:{vz:0.2f}\nphi:{phi:0.2f} theta:{theta:0.2f} psi:{psi:0.2f}")
    return line, arrow, title,

ani = FuncAnimation(fig, animate, frames=int(60*1/dt), interval=1000*dt, blit=True, repeat=False, fargs=(line, title, sys, dt))

plt.show(block=False)
plt.pause(1*60) # Close window after specified number of seconds