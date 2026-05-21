import math


class Localization:
    """
    Maintains robot pose:
    x, y in grid coordinates
    heading in degrees (0° = facing +X direction)
    """

    def __init__(self, grid, start_x=100, start_y=100, start_heading=0):
        self.grid = grid
        self.x = start_x
        self.y = start_y
        self.heading = start_heading

        # For dead-reckoning prediction
        self.last_command = None
        self.speed = 0

    
    # Pose Access
    def get_pose(self):
        return (self.x, self.y, self.heading)


    # Update heading from IMU
    def update_from_imu(self, imu_tuple):
        if imu_tuple is None:
            return

        yaw, pitch, roll = imu_tuple

        self.heading = yaw % 360

   
    # Predict motion based on last movement command
    def predict(self, dt=0.2):
      
        #dt = loop time (seconds)
    
        if self.last_command is None:
            return

        # heading -> radians
        rad = math.radians(self.heading)

        # Movement model
        if self.last_command == "FORWARD":
            dx = math.cos(rad) * self.speed * dt
            dy = math.sin(rad) * self.speed * dt
        elif self.last_command == "BACKWARD":
            dx = -math.cos(rad) * self.speed * dt
            dy = -math.sin(rad) * self.speed * dt
        else:
            dx = dy = 0

        # Update pose
        self.x += dx
        self.y += dy

        # Clamp to grid
        self.x = max(0, min(self.grid.size - 1, int(self.x)))
        self.y = max(0, min(self.grid.size - 1, int(self.y)))


    # Called by MotionController to update movement state
    def notify_motion(self, command, speed=0):
        self.last_command = command
        self.speed = speed