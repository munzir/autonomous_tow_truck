import yaml
import matplotlib.pyplot as plt

# File paths and labels
file_paths = {
    'IMU': 'imu28.txt',
    'Odometry': 'odometry28.txt',
    'GPS': 'gps28.txt',
    'EKF': 'ekf28.txt'
}

# Initialize containers
orientation_z = {}
orientation_w = {}

for label, file in file_paths.items():
    z_vals = []
    w_vals = []
    with open(file, 'r') as f:
        for line in f:
            try:
                data = yaml.safe_load(line)
                # Try to access orientation
                orientation = None
                if isinstance(data, dict):
                    if 'orientation' in data:
                        orientation = data['orientation']
                    elif 'pose' in data:
                        orientation = data.get('pose', {}).get('pose', {}).get('orientation')
                if isinstance(orientation, dict):
                    z_vals.append(orientation.get('z', 0.0))
                    w_vals.append(orientation.get('w', 0.0))
            except Exception as e:
                print(f"Skipping bad line in {label}: {e}")
                continue
    orientation_z[label] = z_vals
    orientation_w[label] = w_vals

# Plotting
plt.figure(figsize=(12, 6))

# orientation.z
plt.subplot(2, 1, 1)
for label, z in orientation_z.items():
    plt.plot(z, label=label)
plt.title('orientation.z over time')
plt.ylabel('z')
plt.legend()
plt.grid(True)

# orientation.w
plt.subplot(2, 1, 2)
for label, w in orientation_w.items():
    plt.plot(w, label=label)
plt.title('orientation.w over time')
plt.ylabel('w')
plt.xlabel('Time Step')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
