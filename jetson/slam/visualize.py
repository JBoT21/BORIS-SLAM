"""
Responsibilities:

1. Display map with Matplotlib

2. Show robot pose

3. Show sensor rays?

4. Save frames for debugging?
"""
import matplotlib.pyplot as plt

def Visualizer(map_grid, robot_pose, sensor_data):
    plt.imshow(map_grid, cmap='gray')  # Occupancy grid
    plt.plot(robot_pose[0], robot_pose[1], 'ro')  # Robot position
    # Optionally plot sensor rays here
    plt.title("SLAM Visualization")
    plt.show()

# Basic visualization function - needs to be tested with actual map and pose data.