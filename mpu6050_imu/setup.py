from glob import glob

from setuptools import setup

package_name = 'mpu6050_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hjt-2',
    maintainer_email='wesleyz@hanjiangtechamr.com',
    description='MPU6050 driver over Linux I2C publishing sensor_msgs/Imu',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'imu_node = mpu6050_imu.imu_node:main',
        ],
    },
)
