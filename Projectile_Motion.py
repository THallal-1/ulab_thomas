#Projectile Motion module
import numpy as np

# Function 1: Calculate the trajectory of the projectile
def calculate_trajectory(initial_velocity, launch_angle, num_points=100):
    """
    Calculate the trajectory of a projectile given its initial velocity and launch angle.

    Inputs:
    initial_velocity (float): Initial velocity of the projectile (in m/s)
    launch_angle (float): Launch angle of the projectile (in degrees)
    num_points (int): Number of time points to calculate along the trajectory (default is 100)

    Outputs:
    trajectory (numpy.ndarray): A 2D array with columns [time, x_position, y_position]
    """
    # Convert launch angle from degrees to radians
    angle_rad = np.radians(launch_angle)
    
    # Gravitational acceleration (m/s^2)
    g = 9.81  
    
    # Time of flight (from the kinematic equation for vertical motion)
    time_of_flight = (2 * initial_velocity * np.sin(angle_rad)) / g
    
    # Handle the case where velocity is too low to take off
    if initial_velocity <= 0:
        raise ValueError("Initial velocity must be greater than 0 to launch.")
    
    # Handle the case where the launch angle is 0 degrees (no vertical motion)
    elif launch_angle == 0:
        print("Warning: Launch angle is 0 degrees, projectile will not rise.")
        return np.zeros((num_points, 3))  # Return all points at height 0
    
    # Handle the case where launch angle is 90 degrees (straight up)
    elif launch_angle == 90:
        print("Warning: Launch angle is 90 degrees, projectile will have no horizontal motion.")
    
    # Time values from 0 to time_of_flight, split into num_points intervals
    time_values = np.linspace(0, time_of_flight, num_points)
    
    # Initialize the trajectory array: time, x_position, y_position
    trajectory = np.zeros((num_points, 3))
    
    for i, t in enumerate(time_values):
        # Horizontal position (constant velocity)
        trajectory[i, 1] = initial_velocity * np.cos(angle_rad) * t
        
        # Vertical position (with gravity)
        trajectory[i, 2] = (initial_velocity * np.sin(angle_rad) * t) - (0.5 * g * t**2)
        
        # Store the time at each point
        trajectory[i, 0] = t

    return trajectory


# Function 2: Calculate the maximum height reached by the projectile
def maximum_height(initial_velocity, launch_angle):
    """
    Calculate the maximum height reached by the projectile.

    Inputs:
    initial_velocity (float): Initial velocity of the projectile (in m/s)
    launch_angle (float): Launch angle of the projectile (in degrees)

    Outputs:
    max_height (float): Maximum height reached by the projectile (in meters)
    """
    # Convert launch angle to radians
    angle_rad = np.radians(launch_angle)
    
    # Gravitational acceleration (m/s^2)
    g = 9.81
    
    # Calculate the maximum height using the vertical component of initial velocity
    max_height = (initial_velocity**2) * (np.sin(angle_rad)**2) / (2 * g)
    
    return max_height


# Function 3: Calculate the horizontal range and time of flight
def horizontal_range_and_time_of_flight(initial_velocity, launch_angle):
    """
    Calculate the horizontal range and time of flight of the projectile.

    Inputs:
    initial_velocity (float): Initial velocity of the projectile (in m/s)
    launch_angle (float): Launch angle of the projectile (in degrees)

    Outputs:
    range (float): Horizontal range of the projectile (in meters)
    time_of_flight (float): Time of flight of the projectile (in seconds)
    """
    # Convert launch angle to radians
    angle_rad = np.radians(launch_angle)
    
    # Gravitational acceleration (m/s^2)
    g = 9.81
    
    # Time of flight calculation
    time_of_flight = (2 * initial_velocity * np.sin(angle_rad)) / g
    
    # Horizontal range calculation (range = velocity * time)
    range = initial_velocity * np.cos(angle_rad) * time_of_flight
    
    return range, time_of_flight


# Example of how you would use the module:

# Initial velocity (m/s) and launch angle (degrees)
initial_velocity = x
launch_angle = y

# Calculate the trajectory
trajectory = calculate_trajectory(initial_velocity, launch_angle)

# Print the first 5 data points of the trajectory
print("First 5 trajectory points (time, x, y):")
print(trajectory[:5])

# Calculate the maximum height
max_height = maximum_height(initial_velocity, launch_angle)
print(f"\nMaximum height: {max_height:.2f} meters")

# Calculate the horizontal range and time of flight
range, time_of_flight = horizontal_range_and_time_of_flight(initial_velocity, launch_angle)
print(f"\nHorizontal range: {range:.2f} meters")
print(f"Time of flight: {time_of_flight:.2f} seconds")
