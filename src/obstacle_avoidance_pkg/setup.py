from setuptools import find_packages, setup

package_name = 'obstacle_avoidance_pkg'

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
    maintainer='salvatore',
    maintainer_email='salvatore.lo.sardo@student.uva.nl',
    description='State machine that follows a reference line and drives around obstacles using the /obj_detect histogram and odometry.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'avoidance = obstacle_avoidance_pkg.main:main',
        ],
    },
)
