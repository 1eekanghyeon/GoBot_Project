from setuptools import find_packages, setup

package_name = 'gps_heading_init'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        ('share/' + package_name + '/launch', ['launch/gps_heading_init.launch.py']),
        # config 파일 설치
        ('share/' + package_name + '/config', ['config/localization_navsat.yaml','config/nav2_params_gps.yaml','config/gps_navigation_bt.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@todo.todo',
    description='GPS-based heading init and re-calibration for Unitree Go2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_heading_init_node = gps_heading_init.gps_heading_init_node:main',
            'imu_bridge = gps_heading_init.imu_bridge:main',
            'gps_path_planner = gps_heading_init.gps_path_planner:main',
            'gps_rtk_pid_nav = gps_heading_init.gps_rtk_pid_nav:main',
        ],
    },
)
