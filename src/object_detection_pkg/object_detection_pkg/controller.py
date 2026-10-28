from setuptools import setup

from object_detection_pkg.object_detection_pkg.velocity_publisher import VelocityPublisher
from object_detection_pkg.object_detection_pkg.lidar_subscribers import LidarSubscriber

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.lidar_subscriber = LidarSubscriber()
        self.velocity_publisher = VelocityPublisher()
    def step(self):
        pass
        
def main():
    print('Hi from object_detection_pkg.')


if __name__ == '__main__':
    main()
