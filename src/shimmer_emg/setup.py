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
        'imu_emg_aq = shimmer_emg.imu_emg_aq:main',
        'record_data = shimmer_emg.record_data:main',
        'plot_data = shimmer_emg.plot_data:main',
        'data_acquisition = shimmer_emg.data_acquisition:main',
        'rp_enable_dc = shimmer_emg.rp_enable_dc:main'
        ],
    },
)
