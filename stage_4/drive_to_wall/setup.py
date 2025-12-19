from setuptools import find_packages, setup

package_name = 'drive_to_wall'

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
    maintainer='root',
    maintainer_email='sk-246767@hs-weingarten.de',
    description='Drive to wall using custom messages',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['drive_to_wall_topic = drive_to_wall.drive_to_wall_topic:main',
        'drive_to_wall_topic_client = drive_to_wall.drive_to_wall_topic_client:main',
        'drive_to_wall_service = drive_to_wall.drive_to_wall_service:main',
        'drive_to_wall_service_client = drive_to_wall.drive_to_wall_service_client:main',
        'drive_to_wall_action = drive_to_wall.drive_to_wall_action:main',
        'drive_to_wall_action_client = drive_to_wall.drive_to_wall_action_client:main',
        ],
    },
)
