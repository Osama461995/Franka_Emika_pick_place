import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import rospy 
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sklearn.decomposition import PCA

# Initialize ROS node
rospy.init_node("ultralytics")
time.sleep(1)

# Set the publishing rate (10 Hz, adjust as needed)
rate = rospy.Rate(10)

# Load the segmentation model
segmentation_model = YOLO("weights_segment/best.pt")

# Publisher for the object info
object_num_pub = rospy.Publisher('/objects_number', String, queue_size=10)  # Publish as a String message
object_pose_pub = rospy.Publisher('/object_poses', PoseStamped, queue_size=20)  # Optionally publish as Pose messages

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

def calculate_3d_coordinates(u, v, depth, fx, fy, cx, cy): 
    """
    Calculate the 3D coordinates (X, Y, Z) from pixel coordinates (u, v) and depth.
    """
    X = ((u - cx) * depth / fx) / 1000
    Y = ((v - cy) * depth / fy) / 1000
    Z = depth / 1000
    return X, Y, Z

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
profile = pipeline.get_active_profile()
depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy = depth_intrinsics.fx, depth_intrinsics.fy
cx, cy = depth_intrinsics.ppx, depth_intrinsics.ppy

# Initialize a list to store the last N angles
angle_history = []
history_length = 10  # Adjust as needed

def moving_average(new_angle):
    angle_history.append(new_angle)
    if len(angle_history) > history_length:
        angle_history.pop(0)
    return np.mean(angle_history)


try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        color_image[:,0:50] = 0
        #croped_image = color_image[0:479,50:639]
        #print(np.shape(croped_image))

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        result = segmentation_model(color_image, conf=0.8, show=True)

        if len(result[0].boxes.cls) > 0:
            num_detected_objects = len(result[0].boxes.cls)
        
        if len(result[0].boxes.cls) > 0:
            for idx, cls in enumerate(result[0].boxes.cls):
                class_index = int(cls.cpu().numpy())
                name = result[0].names[class_index]

                mask = result[0].masks.data.cpu().numpy()[idx, :, :].astype(int)
                nonzero_mask = np.sort(np.nonzero(mask), axis=1)
                x_offset = 10
                y_offset = 10
                mask_border_x = [(nonzero_mask[0][0] - x_offset) if (nonzero_mask[0][0] - x_offset) > 0 else 0, 
                                (nonzero_mask[0][-1] + x_offset) if (nonzero_mask[0][-1] + x_offset) < 480 else 479] 
                mask_border_y = [(nonzero_mask[1][0] - y_offset) if (nonzero_mask[1][0] - y_offset) > 0 else 0, 
                                (nonzero_mask[1][-1] + y_offset) if (nonzero_mask[1][-1] + y_offset) < 640 else 639] 

                # Create a grayscale image from the mask
                grayscale_mask = np.zeros_like(mask, dtype=np.uint8)
                grayscale_mask[mask == 1] = 0  # Black for mask == 1
                grayscale_mask[mask == 0] = 255  # White for mask == 0

                # Find the coordinates of the black pixels (mask == 1)
                black_pixels = np.column_stack(np.where(grayscale_mask == 0))

                # Perform PCA on the black pixel coordinates
                pca = PCA(n_components=2)
                pca.fit(black_pixels)
                orientation_vector = pca.components_[0]  # First principal component

                # Calculate both possible angles
                angle1 = np.arctan2(orientation_vector[1], orientation_vector[0]) * 180 / np.pi
                angle2 = np.arctan2(-orientation_vector[1], -orientation_vector[0]) * 180 / np.pi

                # Ensure angles are in the range [0, 360)
                angle1 = angle1 % 360
                angle2 = angle2 % 360

                # Disambiguate the correct angle based on object shape
                object_height = mask_border_x[1] - mask_border_x[0]
                object_width = mask_border_y[1] - mask_border_y[0]
                aspect_ratio = object_width / object_height

                if aspect_ratio > 1:  # Object is wider than tall
                    correct_angle = angle1 if abs(angle1) < 90 else angle2
                else:  # Object is taller than wide
                    correct_angle = angle1 if abs(angle1) > 90 else angle2

                # Apply moving average to the correct angle
                #smoothed_angle = moving_average(correct_angle)
                correct_angle = correct_angle % 180
                correct_angle = 180 - correct_angle

                print(f"class: {name} Correct orientation angle: {correct_angle}")

                # Vectorized calculation of xyz coordinates
                rows, cols = depth_image.shape
                u, v = np.meshgrid(np.arange(cols), np.arange(rows), indexing='xy')
                depth_values = depth_image[v, u]  # Extract depth values for all pixels
                X = ((u - cx) * depth_values / fx) / 1000
                Y = ((v - cy) * depth_values / fy) / 1000
                Z = depth_values / 1000
                xyz = np.stack((X, Y, Z), axis=-1)  # Combine into (480, 640, 3) array

                # Filter object points using the mask
                #mask[:,0:50] = 0
                object_points = xyz[mask == 1]
                if len(object_points) == 0:
                    continue

                centroid = np.mean(object_points, axis=0)
                quaternion = np.array([0, 0.0, np.sin(-np.pi * 1 / 8 + correct_angle * np.pi / 360), np.cos(-np.pi * 1 / 8 + correct_angle * np.pi / 360)])
                

                pose_msg = PoseStamped()
                pose_msg.header.frame_id = name
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position.x = centroid[0]
                pose_msg.pose.position.y = centroid[1]
                pose_msg.pose.position.z = 0.51
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

                object_pose_pub.publish(pose_msg)
                print(f"{name}: ({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f})")

                objects_num = str(num_detected_objects)
                object_num_pub.publish(objects_num)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()