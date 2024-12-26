import yaml
import matplotlib.pyplot as plt

def load_txt_as_yaml(file_path):
    """Load data from a text file with multiple YAML documents."""
    with open(file_path, 'r') as f:
        documents = list(yaml.safe_load_all(f))  # Load all YAML documents as a list
    return documents

def extract_position(data):
    """Extract only the x and y position from the pose data."""
    # Ensure data is valid and contains the expected structure
    if data is None or 'pose' not in data or 'pose' not in data['pose']:
        return None
    position = data['pose']['pose']['position']
    x = position['x']
    y = position['y']
    return x, y

def plot_positions_line(pos1, pos2):
    """Plot x and y positions for both datasets as line graphs."""
    x1, y1 = zip(*pos1) if pos1 else ([], [])
    x2, y2 = zip(*pos2) if pos2 else ([], [])

    fig, ax = plt.subplots(figsize=(8, 6))

    # Plot X and Y positions for both datasets
    ax.plot(x1, y1, 'r-', label="AMCL Pose (map frame)", marker='o')
    ax.plot(x2, y2, 'b-', label="Odometry Pose (odom frame)", marker='x')

    # Set plot details
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("AMCL vs Odometry Position Comparison (Line Graph mid)")
    ax.legend()
    ax.grid()

    plt.show()

# Load data
file1 = "amcl26.txt"  # Replace with the first file's path
file2 = "wheel26.txt"  # Replace with the second file's path
data1 = load_txt_as_yaml(file1)  # Load all documents for AMCL
data2 = load_txt_as_yaml(file2)  # Load all documents for Odom

# Extract all positions, filtering out invalid entries
positions1 = [extract_position(doc) for doc in data1 if doc is not None and extract_position(doc) is not None]
positions2 = [extract_position(doc) for doc in data2 if doc is not None and extract_position(doc) is not None]

# Plot the positions
plot_positions_line(positions1, positions2)
