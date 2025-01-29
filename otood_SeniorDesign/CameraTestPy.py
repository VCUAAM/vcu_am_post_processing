import pyrealsense2 as rs  # Intel RealSense SDK
import numpy as np
import cv2

# Create a pipeline object for streaming and processing frames
pipeline = rs.pipeline()

# Configure the pipeline to automatically select a device
config = rs.config()

# Optionally specify the stream configuration
# For example, enable depth stream: 640x480 resolution at 30 frames per second (fps)
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
pipeline.start(config)

try:
    # Wait for a coherent set of frames (depth and color) with a timeout of 15 seconds (15000 milliseconds)
    frames = pipeline.wait_for_frames(timeout_ms=15000)

    # Get depth frame
    depth_frame = frames.get_depth_frame()

    if not depth_frame:
        raise Exception("No depth frame captured")

    # Get the depth frame's width and height
    width = depth_frame.get_width()
    height = depth_frame.get_height()

    # Get the distance at the center of the frame
    dist_to_center = depth_frame.get_distance(int(width / 2), int(height / 2))

    # Print the distance to the object in the center
    print(f"The camera is facing an object {dist_to_center:.3f} meters away")

except RuntimeError as e:
    print(f"Error: {e}")
finally:
    # Stop the pipeline when done
    pipeline.stop()
