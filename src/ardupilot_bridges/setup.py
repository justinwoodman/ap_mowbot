from setuptools import find_packages, setup

package_name = 'ardupilot_bridges'

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
    maintainer='ros',
    maintainer_email='justin.woodman@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ap_odometry_bridge = ardupilot_bridges.ap_odometry_bridge:main',
            'ap_navsat_bridge = ardupilot_bridges.ap_navsat_bridge:main',
        ],
    },
)
