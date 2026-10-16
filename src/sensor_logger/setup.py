from setuptools import find_packages, setup

package_name = 'sensor_logger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sensor_logger_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/sensor_logger.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cyrus Chung',
    maintainer_email='cyruschung0603@gmail.com',
    description='Subscribe-only IMU/LiDAR/odometry logger with test-vector categories.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_logger_node = sensor_logger.sensor_logger_node:main',
            'summarize_session = sensor_logger.summarize_session:main',
        ],
    },
)
