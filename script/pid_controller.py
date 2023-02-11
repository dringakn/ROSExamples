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
    
