from setuptools import setup

package_name = "imu_pipeline"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dual_imu_filter.launch.py']),
        ('share/' + package_name + '/config', [
            'config/upper_filter.yaml',
            'config/forearm_filter.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'serial_imu_bridge = imu_pipeline.serial_imu_bridge:main',
            'ble_imu_bridge = imu_pipeline.ble_imu_bridge:main'
        ],
    },
)