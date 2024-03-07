from setuptools import find_packages, setup

package_name = 'shimmer_emg'

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
    maintainer='William',
    maintainer_email='wtolst19@student.aau.dk',
    description='A ROS 2 package for EMG and IMU data acquisition from a Shimmer device.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'emg_aq = shimmer_emg.emg_aq:main',
        'imu_aq = shimmer_emg.imu_aq:main',
        'imu_emg_aq = shimmer_emg.imu_emg_aq:main',
        ],
    },
)
