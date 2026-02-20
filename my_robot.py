from fairis_lib.robot_lib.hambot import HamBot

class MyRobot(HamBot):

    def __init__(self):
        HamBot.__init__(self)
    
    def move_forward(self, velocity=10.0, duration=1000):
        self.set_left_motor_velocity(velocity)
        self.set_right_motor_velocity(velocity)
        
        steps = duration // self.timestep
        for _ in range(steps):
            if self.experiment_supervisor.step(self.timestep) == -1:
                break

        self.stop()

    def move_distance(self, distance, velocity=6.0):
        # Calculate linear velocity: v = ω × r
        linear_velocity = velocity * self.wheel_radius
        
        # Calculate time needed: t = distance / velocity
        time_needed = distance / linear_velocity  # in seconds
        duration = int(time_needed * 1000)  # convert to milliseconds
        
        self.move_forward(velocity, duration)
        

    def turn(self, degrees, velocity=1.0, tolerance=0.1):
        current_angle = self.get_compass_reading()
        target_angle = (current_angle + degrees) % 360
        
        while True:
            current_angle = self.get_compass_reading()
            
            # Calculate shortest angle difference
            angle_diff = target_angle - current_angle
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            
            # Check if we're within tolerance
            if abs(angle_diff) <= tolerance:
                self.stop()
                break
            
            # Turn left (counter-clockwise) or right (clockwise)
            if angle_diff > 0:
                self.set_left_motor_velocity(-velocity)
                self.set_right_motor_velocity(velocity)
            else:
                self.set_left_motor_velocity(velocity)
                self.set_right_motor_velocity(-velocity)
            
            if self.experiment_supervisor.step(self.timestep) == -1:
                break
            
        self.stop()
    
    def wait(self, duration=1000):
        steps = duration // self.timestep
        for _ in range(steps):
            if self.experiment_supervisor.step(self.timestep) == -1:
                break
    
    def drive_circle(self, radius, velocity=6.0, direction='left'):

        import math
        
        if direction == 'left':
            # Left turn: right wheel is outer, left wheel is inner
            outer_velocity = velocity
            inner_velocity = velocity * (radius - self.axel_length / 2) / (radius + self.axel_length / 2)
            self.set_left_motor_velocity(inner_velocity)
            self.set_right_motor_velocity(outer_velocity)
            
            # Calculate based on outer wheel distance (more accurate)
            outer_wheel_circumference = 2 * math.pi * (radius + self.axel_length / 2)
            linear_velocity_outer = outer_velocity * self.wheel_radius
            time_needed = outer_wheel_circumference / linear_velocity_outer
        else:
            # Right turn: left wheel is outer, right wheel is inner
            outer_velocity = velocity
            inner_velocity = velocity * (radius - self.axel_length / 2) / (radius + self.axel_length / 2)
            self.set_left_motor_velocity(outer_velocity)
            self.set_right_motor_velocity(inner_velocity)
            
            # Calculate based on outer wheel distance (more accurate)
            outer_wheel_circumference = 2 * math.pi * (radius + self.axel_length / 2)
            linear_velocity_outer = outer_velocity * self.wheel_radius
            time_needed = outer_wheel_circumference / linear_velocity_outer
        
        duration = round(time_needed * 1000)  # Round duration value
        
        if radius > 1.4:
            duration = int(duration * 1.02)  # Add 2% for large circles
        elif radius < 0.6:
            duration = int(duration * 0.98) # Reduce 2% for small circles
        
        # Execute the circle
        steps = duration // self.timestep
        for _ in range(steps):
            if self.experiment_supervisor.step(self.timestep) == -1:
                break
        
        self.stop()
        
        # Readjust to face north
        current_angle = self.get_compass_reading()
        angle_to_north = 90 - current_angle
        if angle_to_north > 180:
            angle_to_north -= 360
        elif angle_to_north < -180:
            angle_to_north += 360
        
        if abs(angle_to_north) > 0.5:
            self.turn(angle_to_north, tolerance=0.5)
