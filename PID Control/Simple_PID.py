class PID_controller:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        
        self.previous_error = error
        
        return output