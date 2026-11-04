
import math

import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def depth_to_histogram(depth_m, fx, cx, fy, cy, camera_height=0.10,
                       min_obstacle_height=0.03, max_range=10.0,
                       bin_width_deg=5.0):
    """Return distances and angles for angular depth bins."""
    height, width = depth_m.shape

    # middle-lower portion of the image - might need adjustment
    top = int(0.35 * height)
    roi = depth_m[top:int(0.80 * height), :]

    # height of each pixel above the floor (camera assumed level, image y points down)
    rows = np.arange(top, top + roi.shape[0])[:, None]
    with np.errstate(invalid='ignore'):
        above_floor = camera_height - (rows - cy) * roi / fy

    # pixels with a real depth reading (floor and far pixels count as seen, i.e. free)
    measured = np.isfinite(roi) & (roi > 0)

    # we exclude the floor and implausibly distant pixels
    obstacle = np.where(
        measured & (above_floor > min_obstacle_height) & (roi < max_range),
        roi, np.nan
    )

    # + angles are to the left, - anles to the right
    pixel_angles = np.degrees( np.arctan((cx - np.arange(width)) / fx) )

    limit = math.ceil( max(abs(pixel_angles)) / bin_width_deg ) * bin_width_deg

    edges = np.arange( -limit, limit + bin_width_deg, bin_width_deg )

    distances = []
    angles = []

    for start, end in zip(edges[:-1], edges[1:]):
        # pixels whose horizontal angles belong to this bin
        columns = (pixel_angles >= start) & (pixel_angles < end)
        seen = measured[:, columns]
        valid = obstacle[:, columns]
        valid = valid[np.isfinite(valid)]

        # we require sufficient data to trust bin
        if seen.size and seen.sum() >= max(20, 0.1 * seen.size):
            # only floor / far pixels: free up to max range
            distance = float(np.percentile(valid, 10)) if valid.size >= 20 else max_range
        else:
            distance = math.nan  # Unknown, not free

        distances.append(distance)
        angles.append(math.radians((start + end) / 2))

    return distances, angles


class DepthCameraSubscriber(Node):
    def __init__(self):
        super().__init__('depth_camera_subscriber')

        self.bridge = CvBridge()
        self.camera_info = None
        self.distances = None
        self.angles = None
        self.last_frame_time = None
        self.status = 'no depth frame received yet'

        depth_topic = self.declare_parameter( 'depth_topic', '/oak/stereo/image_raw').value
        info_topic = self.declare_parameter( 'camera_info_topic', '/oak/stereo/camera_info').value

        # camera height above the floor in metres (0.10 for the Gazebo ugv_rover)
        self.camera_height = self.declare_parameter('camera_height', 0.10).value
        self.min_obstacle_height = self.declare_parameter('min_obstacle_height', 0.03).value

        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)

    def info_callback(self, msg):
        self.camera_info = msg 

    def depth_callback(self, msg):
        # not reuse old data if this frame is invalid
        self.distances = None
        self.angles = None  

        info = self.camera_info
        if info is None:
            self.set_status('no camera_info received yet')
            return

        if (info.width, info.height) != ( msg.width, msg.height ):
            self.set_status('depth image size does not match camera_info')
            return 

        # Projection matrix for rectified depth images
        if info.p[0] > 0:
            fx, cx, fy, cy = info.p[0], info.p[2], info.p[5], info.p[6]
        else:
            fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
  
        if fx <= 0 or fy <= 0:
            self.set_status('invalid camera intrinsics')
            return  

        if msg.encoding not in ('16UC1', '32FC1'):
            self.set_status(f'expected depth image, got {msg.encoding}')
            return

        try: 
            image = self.bridge.imgmsg_to_cv2( msg, desired_encoding='passthrough')
        except Exception as exc:  
            self.set_status(f'could not decode depth image: {exc}')
            return 

        depth_m = image.astype(np.float32)

        # convert millimetres to metres
        if msg.encoding == '16UC1':
            depth_m /= 1000.0

        distances, angles = depth_to_histogram(
            depth_m, fx, cx, fy, cy,
            camera_height=self.camera_height,
            min_obstacle_height=self.min_obstacle_height
        )

        if any(math.isfinite(d) for d in distances):
            self.distances =distances 
            self.angles =angles 
            self.last_frame_time = self.get_clock().now()
            self.status = 'ok'
        else:
            self.set_status('all depth bins unknown')

    def set_status(self, status):
        self.status = status
        self.get_logger().warn(status, throttle_duration_sec=1.0)

    def read(self):
        if self.distances is None or self.last_frame_time is None:
            return None

        age = (self.get_clock().now() - self.last_frame_time).nanoseconds / 1e9

        if age < 0 or age > 0.5:
            self.set_status(f'depth data stale ({age:.2f} s)')
            return None

        return self.distances

    def get_angles(self):
        return self.angles if self.read() is not None else None
