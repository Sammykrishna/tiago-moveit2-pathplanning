from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='sk-246767@hs-weingarten.de',
    description='Python publisher/subscriber example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_member_function = py_pubsub.publisher_member_function:main',
            'subscriber_member_function = py_pubsub.subscriber_member_function:main',
        ],
    },
)
