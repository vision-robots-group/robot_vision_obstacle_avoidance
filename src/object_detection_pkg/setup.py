from setuptools import find_packages, setup

package_name = 'object_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ionut',
    maintainer_email='ionut.moise@student.uva.nl',
    description='Builds a polar obstacle histogram from LiDAR or depth camera data and publishes it on /obj_detect.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = object_detection_pkg.detection_node:main',
            'depth_detector = object_detection_pkg.detection_node:main_depth',
        ],
    },
)
