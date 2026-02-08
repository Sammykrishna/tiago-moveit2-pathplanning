from setuptools import find_packages, setup

package_name = 'moveit_wrapper_test'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['test_collision_objects = moveit_wrapper_test.test_collision_objects:main',
                            'test_move_robot = moveit_wrapper_test.test_move_robot:main',
                            'test_pick_place = moveit_wrapper_test.test_pick_place:main',
        ],
    },
)
