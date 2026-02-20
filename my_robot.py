from robot_systems.robot import HamBot
import math
import time

class MyRobot(HamBot):

    def __init__(self, lidar_enabled=True, camera_enabled=True):
        super().__init__(lidar_enabled=lidar_enabled, camera_enabled=camera_enabled)

        # Physical Robot Specifications
        self.wheel_radius = 0.045   # meters
        self.axel_length = 0.184    # meters
        self.max_motor_speed = 100  # speed cap

    def get_compass_reading(self):
        """Return heading from the IMU in degrees (0–360)."""
        bearing = self.get_heading()
        if bearing < 0.0:
            bearing += 360.0
        return round(bearing)
    
    def get_lidar_range_image(self):
        """Return 360° range scan from the Lidar (mm)."""
        if self.lidar is None:
            raise RuntimeError("Lidar is not enabled on this MyRobot instance.")
        return self.get_range_image()

    def get_encoder_readings(self):
        """Return [left, right] encoder values."""
        return [self.left_motor.get_encoder(), self.right_motor.get_encoder()]

    def get_left_motor_encoder_reading(self):
        return self.left_motor.get_encoder()

    def get_right_motor_encoder_reading(self):
        return self.right_motor.get_encoder()

    def velocity_saturation(self, speed, suppress=False):
        """Clamp speed to ±max_motor_speed."""
        if speed > self.max_motor_speed:
            if not suppress:
                print("Motor speed capped to:", self.max_motor_speed)
            return self.max_motor_speed
        elif speed < -self.max_motor_speed:
            if not suppress:
                print("Motor speed capped to:", -self.max_motor_speed)
            return -self.max_motor_speed
        return speed

    def set_left_motor_velocity(self, speed, suppress=False):
        """Set left motor speed (left motor is reversed on hardware)."""
        self.set_left_motor_speed(self.velocity_saturation(speed, suppress=suppress))

    def set_right_motor_velocity(self, speed, suppress=False):
        """Set right motor speed."""
        self.set_right_motor_speed(self.velocity_saturation(speed, suppress=suppress))

    def stop(self):
        """Stop all motors."""
        self.stop_motors()

    def go_forward(self, speed=50):
        """Drive forward."""
        self.set_left_motor_speed(self.velocity_saturation(speed))
        self.set_right_motor_speed(self.velocity_saturation(speed))

    def move_forward(self, speed=50, duration=1.0):
        """Drive straight for *duration* seconds."""
        self.set_left_motor_speed(speed)
        self.set_right_motor_speed(speed)
        time.sleep(duration)
        self.stop_motors()

    def move_distance(self, distance, speed=50):
        """Drive forward a specified distance (meters)."""
        linear_velocity = speed * self.wheel_radius
        time_needed = distance / linear_velocity  # seconds
        self.move_forward(speed, time_needed)

    def turn(self, degrees, speed=20, tolerance=0.1):
        """Turn in-place by *degrees* using the IMU for feedback."""
        current_angle = self.get_compass_reading()
        target_angle = (current_angle + degrees) % 360

        while True:
            current_angle = self.get_compass_reading()

            # Shortest angle difference
            angle_diff = target_angle - current_angle
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360

            if abs(angle_diff) <= tolerance:
                self.stop_motors()
                break

            # (-speed, +speed) = spin left (CCW, heading increases)
            # (+speed, -speed) = spin right (CW, heading decreases)
            if angle_diff > 0:          # need to increase heading → spin left
                self.set_left_motor_speed(-speed)
                self.set_right_motor_speed(speed)
            else:                       # need to decrease heading → spin right
                self.set_left_motor_speed(speed)
                self.set_right_motor_speed(-speed)

            time.sleep(0.01)

        self.stop_motors()

    def wait(self, duration=1.0):
        """Pause execution for *duration* seconds."""
        time.sleep(duration)

    def drive_circle(self, radius, speed=50, direction='left'):
        """Drive a full circle of the given radius, then re-align to north."""

        inner_speed = speed * (radius - self.axel_length / 2) / (radius + self.axel_length / 2)

        if direction == 'left':
            # Left turn: right wheel outer, left wheel inner (left negated)
            self.set_left_motor_speed(-inner_speed)
            self.set_right_motor_speed(speed)
        else:
            # Right turn: left wheel outer, right wheel inner
            self.set_left_motor_speed(-speed)
            self.set_right_motor_speed(inner_speed)

        # Time to complete one full circle (outer wheel)
        outer_circumference = 2 * math.pi * (radius + self.axel_length / 2)
        linear_velocity_outer = speed * self.wheel_radius
        time_needed = outer_circumference / linear_velocity_outer

        # Radius-based corrections
        if radius > 1.4:
            time_needed *= 1.02
        elif radius < 0.6:
            time_needed *= 0.98

        time.sleep(time_needed)
        self.stop_motors()

        # Re-adjust to face north
        current_angle = self.get_compass_reading()
        angle_to_north = 90 - current_angle
        if angle_to_north > 180:
            angle_to_north -= 360
        elif angle_to_north < -180:
            angle_to_north += 360

        if abs(angle_to_north) > 0.5:
            self.turn(angle_to_north, tolerance=0.5)


if __name__ == "__main__":
    robot = MyRobot(lidar_enabled=False, camera_enabled=False)

    try:

        # Drive forward for 2 seconds
        print("Moving forward...")
        robot.move_forward(speed=25, duration=2.0)
        robot.wait(0.5)

        # Turn 90° to the right
        print("Turning 90° right...")
        robot.turn(-90)
        print("Heading after right turn:", robot.get_compass_reading(), "°")
        robot.wait(0.5)

        # Drive forward again
        print("Moving forward...")
        robot.move_forward(speed=25, duration=2.0)
        robot.wait(0.5)

        # Turn 90° to the left
        print("Turning 90° left...")
        robot.turn(90)
        print("Heading after left turn:", robot.get_compass_reading(), "°")
        robot.wait(0.5)

        # Drive forward 0.5 meters
        print("Moving 0.5 m forward...")
        robot.move_distance(0.5)

        print("Done! Final heading:", robot.get_compass_reading(), "°")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        robot.stop_motors()
        print("Motors stopped.")

