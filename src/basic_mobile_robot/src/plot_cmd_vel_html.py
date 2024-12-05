import re
import plotly.graph_objects as go

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

# Create the plot
fig = go.Figure()

# Add target_linear to the plot
fig.add_trace(go.Scatter(
    x=timestamps,
    y=target_linear,
    mode='lines',
    name='Target Linear',
    line=dict(color='blue')
))

# Add target_rot to the plot
fig.add_trace(go.Scatter(
    x=timestamps,
    y=target_rot,
    mode='lines',
    name='Target Rotational',
    line=dict(color='red')
))

# Customize the layout
fig.update_layout(
    title="Target Linear and Rotational Values",
    xaxis_title="Timestamp (seconds)",
    yaxis_title="Values",
    legend=dict(x=0.05, y=0.95),
    template="plotly_white",
)

# Save the plot to an HTML file
output_file = "plot.html"
fig.write_html(output_file)

print(f"Plot saved to {output_file}. Open it in a browser to view the interactive graph.")

