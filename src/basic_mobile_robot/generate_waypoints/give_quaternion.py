import math

def calculate_quaternion(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1
    theta = math.atan2(delta_y, delta_x)
    
    q_z = math.sin(theta / 2)
    q_w = math.cos(theta / 2)
    
    return q_z, q_w

# Example usage
x1, y1 = 31.4539, 21.1372 
x2, y2 = 24.4884, 19.5577

q_z, q_w = calculate_quaternion(x1, y1, x2, y2)
print(f"Quaternion: z={q_z}, w={q_w}")

