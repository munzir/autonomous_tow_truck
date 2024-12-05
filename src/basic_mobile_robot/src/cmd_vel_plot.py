import re
import matplotlib.pyplot as plt

# File containing the log data
log_file = "/root/cmd_vel"

# Initialize dictionaries to store data
data = {
    "timestamps": [],
    "target_linear": [],
    "target_rot_": [],
    "target_rot": [],
    "wheel_separation_": [],
    "tanSteer": [],
    "target_left_steering": [],
    "left_steering_angle": [],
    "left_steering_diff": [],
    "left_steering_cmd": [],
}

# Regex to extract the data
regex = (
    r"\[WARN\] \[([\d.]+)\] \[gazebo_ros_ackermann_drive\]: steering command: "
    r"target_linear=([\d.-]+), target_rot_=([\d.-]+), target_rot=([\d.-]+), "
    r"wheel_separation_=([\d.-]+), tanSteer=([\d.-]+), "
    r"target_left_steering=([\d.-]+), left_steering_angle=([\d.-]+), "
    r"left_steering_diff=([\d.-]+), left_steering_cmd=([\d.-]+)"
)

# Parse the file
with open(log_file, "r") as file:
    for line in file:
        match = re.search(regex, line)
        if match:
            data["timestamps"].append(float(match.group(1)))
            data["target_linear"].append(float(match.group(2)))
            data["target_rot_"].append(float(match.group(3)))
            data["target_rot"].append(float(match.group(4)))
            data["wheel_separation_"].append(float(match.group(5)))
            data["tanSteer"].append(float(match.group(6)))
            data["target_left_steering"].append(float(match.group(7)))
            data["left_steering_angle"].append(float(match.group(8)))
            data["left_steering_diff"].append(float(match.group(9)))
            data["left_steering_cmd"].append(float(match.group(10)))

# Create subplots
plt.figure(figsize=(15, 10))

# Plot each parameter
for i, (key, values) in enumerate(data.items()):
    if key != "timestamps":  # Exclude timestamps from y-axis data
        plt.subplot(3, 3, i)  # Create a 3x3 grid of subplots
        plt.plot(data["timestamps"], values, label=key.replace("_", " ").title())
        plt.title(key.replace("_", " ").title())
        plt.xlabel("Timestamp (seconds)")
        plt.ylabel("Value")
        plt.legend()
        plt.grid()

# Adjust layout and show plot
plt.tight_layout()
plt.show()

