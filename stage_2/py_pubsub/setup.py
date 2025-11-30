from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='samanth krishna',
    maintainer_email='sk-246767@hs-weingarten.de',
    description='Simple ROS 2 pub/sub example in one file',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'publisher_member_function = py_pubsub.pubsub:main_publisher',
            'subscriber_member_function = py_pubsub.pubsub:main_subscriber',
        ],
    },
)
