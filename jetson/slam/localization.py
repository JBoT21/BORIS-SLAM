import math


class Localization:
    """
    Maintains robot pose:
    x, y in continuous coordinates (floats)
    heading in degrees (0° = +X direction)
    """

    def __init__(self, grid, start_x=100, start_y=100, start_heading=0):
        self.grid = grid
        # Continuous pose
        self.x = float(start_x)
        self.y = float(start_y)
        self.heading = float(start_heading)
        # Motion state
        self.last_command = "S"
        self.speed = 0.0
        # IMU smoothing
        self.alpha = 0.6   # complementary filter weight

   #Pose check
    def get_pose(self):
        return (int(self.x), int(self.y), self.heading)

    #IMU update
    def update_from_imu(self, imu_tuple):
        if imu_tuple is None:
            return
        yaw, pitch, roll = imu_tuple
        self.heading = (self.alpha * yaw + (1 - self.alpha) * self.heading) % 360

    def predict(self, dt=0.2):
        #Predict motion based on last command.
        #dt = loop time (seconds)
        

        rad = math.radians(self.heading)
        # Movement model
        if self.last_command == "F":
            dx = math.cos(rad) * self.speed * dt
            dy = math.sin(rad) * self.speed * dt

        elif self.last_command == "B":
            dx = -math.cos(rad) * self.speed * dt
            dy = -math.sin(rad) * self.speed * dt

        else:
            dx = dy = 0.0

        # Update continuous pose
        self.x += dx
        self.y += dy

        # Clamp to grid bounds (float-safe)
        self.x = max(0.0, min(self.grid.size - 1, self.x))
        self.y = max(0.0, min(self.grid.size - 1, self.y))

  
    def notify_motion(self, command, speed=0):
        """
        Called by MotionController.
        command: 'F', 'B', 'L', 'R', or 'S'
        speed: 0-255 (PWM)
        """
        self.last_command = command
        # Convert PWM to approximate grid speed
        # (I will calibrate this later)
        self.speed = speed / 255.0 * 5.0  # Currently assuming max 5 grid units/sec
        # Turning commands do not move the robot
        if command in ("L", "R", "S"):
            self.speed = 0.0