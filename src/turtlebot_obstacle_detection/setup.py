from setuptools import find_packages, setup

package_name = 'turtlebot_obstacle_detection'

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
    maintainer='charuka',
    maintainer_email='chrk4.pro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector = turtlebot_obstacle_detection.obstacle_detector:main',
            'object_recognizer = turtlebot_obstacle_detection.object_recognizer_nav:main',
            'autonomous_nav2_explorer = turtlebot_obstacle_detection.autonomous_nav2_explorer:main',
            'teleop_wasd = turtlebot_obstacle_detection.teleop_wasd:main',
            'image_obstacle_simulator = turtlebot_obstacle_detection.image_obstacle_simulator:main',
        ],
    },
)
