import matplotlib.pyplot as plt
import numpy as np
# Importing the necessary modules
import math
# Constants
L1 = 10.5  # Length of the first link (cm)
L2 = 15.0  # Length of the second link (cm)
OFFSET_Y = 10.0

def computeIK_with_offset(x, y):
    y = y - OFFSET_Y  # Adjusting for the offset in y-coordinate
    d = math.sqrt(x**2 + y**2)
    theta2_rad = math.acos((d**2 - L1**2 - L2**2) / (2 * L1 * L2))
    alpha = math.atan2(y, x)
    beta = math.asin(L2 * math.sin(theta2_rad) / d)
    theta1_rad = alpha - beta

    # Convert from radians to degrees
    theta1_deg = math.degrees(theta1_rad) + 180
    theta2_deg = math.degrees(theta2_rad) + 180
    return theta1_deg, theta2_deg

def plot_robot_arm_with_offset(x, y):
    # Compute inverse kinematics
    theta1_deg, theta2_deg = computeIK_with_offset(x, y)
    
    # Convert degrees to radians
    theta1_rad = np.radians(theta1_deg - 180)
    theta2_rad = np.radians(theta2_deg - 180)
    
    # Calculate positions using forward kinematics
    x1 = L1 * np.cos(theta1_rad)
    y1 = OFFSET_Y + L1 * np.sin(theta1_rad)  # Adjusting for the offset
    
    x2 = x1 + L2 * np.cos(theta1_rad + theta2_rad)
    y2 = y1 + L2 * np.sin(theta1_rad + theta2_rad)
    
    # Plotting
    plt.figure(figsize=(10, 8))
    
    # Plot base
    plt.scatter(0, OFFSET_Y, color='red', s=100, label='Base')  # Adjusted for the offset
    

    # Plot link 1
    plt.plot([0, x1], [OFFSET_Y, y1], 'o-', color='blue', markersize=10, label='Link 1')  # Adjusted for the offset
    
    # Plot link 2
    plt.plot([x1, x2], [y1, y2], 'o-', color='green', markersize=10, label='Link 2')
    
    # Plot target point
    plt.scatter(x, y, color='purple', s=100, marker='x', label='Target')
    
    plt.xlim(-L1 - L2, L1 + L2)
    plt.ylim(-L1, 2*L1 + L2)  # Adjusted range for the offset
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title(f"Robot Arm Position for Target ({x}, {y}) with Offset")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)
    plt.show()

# Plot robot arm for a sample position
x_sample = 10.5
y_sample = 0
# Plot robot arm for a sample position with the offset
plot_robot_arm_with_offset(x_sample, y_sample)
