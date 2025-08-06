from setuptools import setup #,find_packages

package_name = 'post_stations'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    #find_packages(exclude=['test']),
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Reusable base station logic for POST system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loopback_station = post_stations.loopback_station:main'
        ]
    }
)
