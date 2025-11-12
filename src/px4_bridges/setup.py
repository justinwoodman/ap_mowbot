from setuptools import find_packages, setup

package_name = 'px4_bridges'

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
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry_bridge = px4_bridges.odometry_bridge:main',
            'navsat_bridge = px4_bridges.navsat_bridge:main',
            'cmd_vel_bridge = px4_bridges.cmd_vel_bridge:main',
            'ntrip_bridge = px4_bridges.ntrip_bridge:main'
        ],
    },
)
