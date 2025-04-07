import yaml
import matplotlib.pyplot as plt

def read_sec_timestamps(filename):
    with open(filename, 'r') as file:
        docs = list(yaml.safe_load_all(file))

    sec_timestamps = []
    for doc in docs:
        if doc is None:
            continue
        sec = doc.get('header', {}).get('stamp', {}).get('sec', None)
        if sec is not None:
            sec_timestamps.append(sec)
    return sec_timestamps

# File paths
imu_file = 'imugps.txt'
gps_file = 'gps.txt'

imu_secs = read_sec_timestamps(imu_file)
gps_secs = read_sec_timestamps(gps_file)

# Plotting
plt.plot(imu_secs, label='IMU Secs', marker='o')
plt.plot(gps_secs, label='GPS Secs', marker='x')
plt.xlabel('Message Index')
plt.ylabel('Timestamp (sec)')
plt.title('IMU vs GPS Sec Field Comparison')
plt.legend()
plt.grid(True)
plt.show()

# Optional: Print timestamp differences by index
print("\n--- Timestamp Differences (IMU - GPS) ---")
min_len = min(len(imu_secs), len(gps_secs))
for i in range(min_len):
    print(f"Entry {i}: {imu_secs[i]} - {gps_secs[i]} = {imu_secs[i] - gps_secs[i]} sec")
