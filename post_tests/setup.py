from setuptools import setup

package_name = 'post_tests'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/post_tests/launch', ['launch/dynamic_loop_test_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loopback_station = post_stations.loopback_station:main',
            'two_station_test = post_stations.two_station_test:main',
            'sender_receiver_test = post_tests.sender_receiver_test:main',
            'dynamic_loop_publisher = post_tests.dynamic_loop_publisher:main'
        ],
    },
)
