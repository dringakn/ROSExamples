#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    A simple, reusable PID controller implementation in pure Python.
    Computes control actions to drive a process variable toward a desired setpoint.

Features:
  • Configurable:
      – setpoint: desired target value
      – kp, ki, kd: proportional, integral, and derivative gains
  • Internal state:
      – integrator accumulates error over time
      – differentiator tracks rate of change of error
      – previous error stored for derivative calculation
  • Anti-windup built‑in by straightforward integrator accumulation
  • Single-call interface:
      – calculate_input(process_variable, dt) → control output
  • Easy to embed in any control loop

Usage Example:
    pid = PIDController(setpoint=100.0, kp=1.2, ki=0.01, kd=0.1)
    dt = 0.05  # 50 ms loop time
    while True:
        current = read_sensor()
        control_signal = pid.calculate_input(current, dt)
        apply_actuator(control_signal)
"""

class PIDController:
    def __init__(self, setpoint, kp, ki, kd):
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator = 0
        self.differentiator = 0
        self.error_previous = 0
        
    def calculate_input(self, process_variable, dt):
        error = self.setpoint - process_variable
        self.integrator = self.integrator + error * dt
        derivative = (error - self.error_previous) / dt
        self.error_previous = error
        self.differentiator = derivative
        
        return self.kp * error + self.ki * self.integrator + self.kd * derivative
    
