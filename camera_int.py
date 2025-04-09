import pyrealsense2 as rs

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)

# Get the depth stream intrinsics
depth_sensor = profile.get_stream(rs.stream.depth)
intrinsics = depth_sensor.as_video_stream_profile().get_intrinsics()

print(f"fx: {intrinsics.fx}")
print(f"fy: {intrinsics.fy}")

# Stop streaming
pipeline.stop()
