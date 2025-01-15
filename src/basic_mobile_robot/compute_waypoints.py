import sys
import math
import csv

def read_input_file(filename):
    """Read input parameters from a file."""
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) != 6:
                raise ValueError("Input file must contain exactly five parameters: x, y, w, z, total_distance, distance_between_waypoints")
            x, y, w, z, total_distance, distance_between_waypoints = map(float, row)
            return x, y, w, z, total_distance, distance_between_waypoints


def generate_waypoints(x, y, w, z, total_distance, distance_between_waypoints):
    """Generate waypoints along a straight line based on the initial pose and distance parameters."""
    # Calculate the direction vector from the quaternion
    angle = 2 * math.atan2(z, w)
    dx = math.cos(angle)
    dy = math.sin(angle)

    # Number of waypoints to generate
    num_waypoints = int(total_distance // distance_between_waypoints)

    waypoints = []
    for i in range(num_waypoints + 1):
        new_x = x + i * distance_between_waypoints * dx
        new_y = y + i * distance_between_waypoints * dy
        waypoints.append((new_x, new_y, w, z))

    return waypoints


def print_waypoints(waypoints):
    """Print the waypoints to stdout."""
    for waypoint in waypoints:
        print(f"{waypoint[0]},{waypoint[1]},{waypoint[2]},{waypoint[3]}")


def main():
    if len(sys.argv) != 2:
        print("Usage: python compute_waypoints.py <input_file>")
        return

    input_file = sys.argv[1]

    try:
        x, y, w, z, total_distance, distance_between_waypoints = read_input_file(input_file)
        waypoints = generate_waypoints(x, y, w, z, total_distance, distance_between_waypoints)
        print_waypoints(waypoints)
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()

