from setuptools import find_packages, setup

package_name = 'imu_pipeline'

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
    maintainer='ee106a-abg',
    maintainer_email='nicholasqb01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_imu_bridge = imu_pipeline.serial_imu_bridge:main',
            'ble_imu_bridge = imu_pipeline.ble_imu_bridge:main'
        ],
    },
)