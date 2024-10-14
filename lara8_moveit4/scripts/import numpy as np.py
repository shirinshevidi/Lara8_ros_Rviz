import numpy as np
import tf.transformations as tf

# Define the Euler angles for element3 (in degrees)
A, B, C = 90, -90 , 0 # These are the initial roll, pitch, yaw angles

# Convert degrees to radians
roll, pitch, yaw = np.deg2rad([A, B, C])

# Convert the initial Euler angles to a quaternion
quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

# Function to calculate the Y-axis vector from a quaternion
def quaternion_to_y_axis(qx, qy, qz, qw):
    y_x = 2 * (qx * qy - qz * qw)
    y_y = 1 - 2 * (qx**2 + qz**2)
    y_z = 2 * (qy * qz + qx * qw)
    return np.array([y_x, y_y, y_z])

# Get the Y-axis vector for the given quaternion
y_axis = quaternion_to_y_axis(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
print("Y-axis vector:", y_axis)

# Define the angle by which you want to rotate around the Y-axis (in degrees)
rotation_angle_degrees = -90
rotation_angle_radians = np.deg2rad(rotation_angle_degrees)

# Create the quaternion for a rotation around the Y-axis
rotation_quaternion = tf.quaternion_about_axis(rotation_angle_radians, y_axis)  # Rotate around the global Y-axis

# Combine the original quaternion with the new rotation
new_quaternion = tf.quaternion_multiply(rotation_quaternion, quaternion)

# Convert the new quaternion to Euler angles (in radians)
new_euler = tf.euler_from_quaternion(new_quaternion)

# Convert the new Euler angles back to degrees
new_euler_degrees = np.rad2deg(new_euler)

# Print the new Euler angles in degrees and the new position (which remains the same)
X, Y, Z = 0.83, 0.1, 0.78
print("New orientation (degrees):", new_euler_degrees)
print(f"New element3 = ({new_euler_degrees[0]}, {new_euler_degrees[1]}, {new_euler_degrees[2]}, {X}, {Y}, {Z})")
