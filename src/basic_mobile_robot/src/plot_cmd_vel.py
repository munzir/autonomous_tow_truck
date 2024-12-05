import re
import matplotlib.pyplot as plt

# File containing the log data
log_file = "/root/cmd_vel"

# Initialize lists to store parsed data
timestamps = []
target_linear = []
target_rot = []

# Regex to extract the data
regex = r"\[WARN\] \[([\d.]+)\] \[gazebo_ros_ackermann_drive\]: target_linear=([\d.-]+), target_rot_=([\d.-]+)"

# Parse the file
with open(log_file, "r") as file:
    for line in file:
        match = re.search(regex, line)
        if match:
            timestamps.append(float(match.group(1)))
            target_linear.append(float(match.group(2)))
            target_rot.append(float(match.group(3)))

# Plot the data
plt.figure(figsize=(10, 5))

# Plot target_linear
plt.plot(timestamps, target_linear, label="Target Linear", color="blue")

# Plot target_rot
plt.plot(timestamps, target_rot, label="Target Rotational", color="red")

# Add labels and title
plt.xlabel("Timestamp (seconds)")
plt.ylabel("Values")
plt.title("Target Linear and Rotational Values")
plt.legend()
plt.grid(True)

# Show the plot
plt.tight_layout()
plt.show()

