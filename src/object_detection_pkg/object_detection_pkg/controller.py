import math

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from object_detection_pkg.lidar_subscribers import LidarSubscriber
from object_detection_pkg.depth_camera_subscriber import DepthCameraSubscriber



class Controller(Node):
    def __init__(self, sensors, safe_distance=1.0, danger_zone_factor=1.2):
        super().__init__('controller')
        self.sensors = sensors
        self.safe_distance = self.positive_param('safe_distance', safe_distance)
        self.danger_zone = self.safe_distance * danger_zone_factor # margin to look for objects a bit further as well since robot is moving
        self.fov = math.radians(10)
        self.bin_width = math.radians(5)
        self.robot_width = 0.255 + 0.01# 255mm + 0.01 margin
        # tghe depth cam reports forward depth; lidar reports radial dist
        self.is_depth = isinstance(sensors, DepthCameraSubscriber)
        self.publisher = self.create_publisher( Float32MultiArray, 'obj_detect', 10 )
        self.timer = self.create_timer(0.1, self.step)

    def positive_param(self, name, default):
        value = float(self.declare_parameter(name, float(default)).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive, got {value}')
        return value

    def make_histogram(self, distances, angles):
        count = round(2 * self.fov / self.bin_width)
        values = [math.nan] * count
        centers = [
            -self.fov + (i + 0.5) * self.bin_width
            for i in range(count)
        ]

        for d, a in zip(distances, angles):
            # we ignore empty / invalid readings
            if not math.isfinite(d) or d <= 0:
                continue
            if not math.isfinite(a):
                continue

            # norm angles to [-pi, pi].
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) > self.fov:
                continue

            index = min( int((a + self.fov) / self.bin_width), count - 1)

            # nearest reading in each angular bin
            if not math.isfinite(values[index]) or d < values[index]:
                values[index] = float(d)

        return list(zip(values, centers))

    def point(self, distance, angle):
        if self.is_depth:
            return distance * math.tan(angle), distance

        return distance * math.sin(angle), distance * math.cos(angle)

    def fill_small_gaps(self, front):
        # postprocessing
        # we define consecutive close bins to belong to the same object
        objects = []

        for index, (distance, _) in enumerate(front):
            if not math.isfinite(distance):
                continue
            if distance >= self.danger_zone:
                continue

            if objects and objects[-1][-1] == index - 1: # processing only objects that are dangerous
                objects[-1].append(index)
            else:
                objects.append([index])

        # boundaries of neighboring objects
        for left, right in zip(objects, objects[1:]):
            left_index = left[-1]
            right_index = right[0]

            left_distance, left_angle = front[left_index]
            right_distance, right_angle = front[right_index]

            left_x, left_y = self.point(left_distance, left_angle)
            right_x, right_y = self.point(right_distance, right_angle)

            gap = math.hypot(right_x - left_x, right_y - left_y) # dist btw 2 objs

            if gap <= self.robot_width: # we check if the gap btw obj is enough for the robot to pass or not
                distance = min(left_distance, right_distance)

                for index in range(left_index + 1, right_index):
                    _, angle = front[index]
                    front[index] = (distance, angle)

        return front

    def step(self):
        distances = self.sensors.read()
        angles = self.sensors.get_angles()

        if distances is None or angles is None:
            status = getattr(self.sensors, 'status', 'unknown')
            self.get_logger().warn(f'No sensor data ({status})', throttle_duration_sec=1.0 )

            # we publish unknown bins instead of implying a clear path
            distances, angles = [], []

        front = self.make_histogram(distances, angles)
        front = self.fill_small_gaps(front)

        self.publish_histogram(front)

    def publish_histogram(self, front):
        msg = Float32MultiArray()

        # Rows are bins and columns are [distance, angle]
        msg.layout.dim = [
            MultiArrayDimension(label='bins', size=len(front), stride=len(front)*2),
            MultiArrayDimension(label='distance_m_angle_rad', size=2, stride=2),
        ]
        msg.data = [float(value) for distance, angle in front for value in (distance, angle)]

        self.publisher.publish(msg)


def _run(sensor_class, args=None):
    rclpy.init(args=args)
    
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    safe_distance = float(config["safe_distance"])
    danger_zone_factor = float(config["danger_zone_factor"])
    
    sensor = sensor_class()
    controller = Controller(sensor, safe_distance=safe_distance, danger_zone_factor=danger_zone_factor)

    executor = SingleThreadedExecutor()
    for node in (controller, sensor):
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()

        for node in (controller, sensor):
            node.destroy_node()

        rclpy.shutdown()


def main(args=None):
    _run(LidarSubscriber, args)


def main_depth(args=None):
    _run(DepthCameraSubscriber, args)


if __name__ == '__main__':
    main()