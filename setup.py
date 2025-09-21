from setuptools import find_packages, setup

package_name = 'warmup_project'

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
    maintainer='grant',
    maintainer_email='gsullivanr@icloud.com',
    description='later',
    license='later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_node = warmup_project.circle_node:main',
            'teleop = warmup_project.teleop:main',
            'wall_follower = warmup_project.wall_follower:main',
            'controller = warmup_project.controller:main',
            'spiral = warmup_project.spiral:main',
            'person_follower = warmup_project.person_follower:main',
            'turn_around = warmup_project.turn_around:main',
        ],
    },
)
