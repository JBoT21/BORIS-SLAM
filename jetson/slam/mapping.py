"""
Responsibilities:

1. Sweep servo angles

2. Convert distance + angle → map coordinates

3. Update occupancy grid

4. Handle unknown space vs free space

(Most previously adapted code will go here)
"""
import numpy as np

def MappingEngine(occupancy_grid, robot_pose, sensor_data):

    # Unpack robot pose
    x, y, heading = robot_pose
    
    # Process sensor data (assuming it's a list of (angle, distance) tuples)
    for angle, distance in sensor_data:
        # Convert angle to radians and adjust for robot's heading
        total_angle = np.radians(angle + heading)
        
        # Calculate the coordinates of the detected obstacle
        obs_x = int(x + distance * np.cos(total_angle))
        obs_y = int(y + distance * np.sin(total_angle))
        
        # Update occupancy grid (mark as occupied)
        if 0 <= obs_x < occupancy_grid.shape[0] and 0 <= obs_y < occupancy_grid.shape[1]:
            occupancy_grid[obs_y, obs_x] = 1  # Mark as occupied
            
            # Optionally mark free space along the ray
            for i in range(1, int(distance)):
                free_x = int(x + i * np.cos(total_angle))
                free_y = int(y + i * np.sin(total_angle))
                if 0 <= free_x < occupancy_grid.shape[0] and 0 <= free_y < occupancy_grid.shape[1]:
                    occupancy_grid[free_y, free_x] = -1  # Mark as free

    return occupancy_grid

#Return to later