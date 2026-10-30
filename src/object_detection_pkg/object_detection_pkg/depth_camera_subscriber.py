
import math

import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def depth_to_histogram(depth_m, fx, cx, bin_width_deg=5.0):
    """Return distances and angles for angular depth bins."""
    height, width = depth_m.shape

    # middle-lower portion of the image - might need adjustment
    roi = depth_m[int(0.35 * height):int(0.80 * height), :]

    # + angles are to the left, - anles to the right
    pixel_angles = np.degrees( np.arctan((cx - np.arange(width)) / fx) )

    limit = math.ceil( max(abs(pixel_angles)) / bin_width_deg ) * bin_width_deg

    edges = np.arange( -limit, limit + bin_width_deg, bin_width_deg )

    distances = []
    angles = []

    for start, end in zip(edges[:-1], edges[1:]):
        # pixels whose horizontal angles belong to this bin
        sector = roi[:, (pixel_angles >= start) & (pixel_angles < end)]

        # we exclude invalid and implausibly distant pixels
        valid = sector[
            np.isfinite(sector) &
            (sector > 0) &
            (sector < 10.0)
        ]

        # we require sufficient data to trust bin
        if sector.size and valid.size >= max(20, 0.1 * sector.size):
            distance = float(np.percentile(valid, 10))
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

        depth_topic = self.declare_parameter( 'depth_topic', '/oak/stereo/image_raw').value
        info_topic = self.declare_parameter( 'camera_info_topic', '/oak/stereo/camera_info').value

        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)

    def info_callback(self, msg):
        self.camera_info = msg 

    def depth_callback(self, msg):
        # not reuse old data if this frame is invalid
        self.distances = None
        self.angles = None  

        info = self.camera_info
        if info is None or (info.width, info.height) != ( msg.width, msg.height ):
            return 

        # Projection matrix for rectified depth images
        if info.p[0] > 0:
            fx, cx = info.p[0], info.p[2]
        else:
            fx, cx = info.k[0], info.k[2]
  
        if fx <= 0:
            return  

        if msg.encoding not in ('16UC1', '32FC1'):
            self.get_logger().error(f'Expected depth image, got {msg.encoding}', throttle_duration_sec=5.0)
            return

        try: 
            image = self.bridge.imgmsg_to_cv2( msg, desired_encoding='passthrough')
        except Exception as exc:  
            self.get_logger().error(f'Could not decode depth image: {exc}', throttle_duration_sec=5.0 )
            return 

        depth_m = image.astype(np.float32)

        # convert millimetres to metres
        if msg.encoding == '16UC1':
            depth_m /= 1000.0

        distances, angles = depth_to_histogram(depth_m, fx, cx)

        if any(math.isfinite(d) for d in distances):
            self.distances =distances 
            self.angles =angles 
            self.last_frame_time = self.get_clock().now()

    def read(self):
        if self.distances is None or self.last_frame_time is None:
            return None

        age = (self.get_clock().now() - self.last_frame_time).nanoseconds / 1e9

        if age < 0 or age > 0.5:
            return None

        return self.distances

    def get_angles(self):
        return self.angles if self.read() is not None else None
