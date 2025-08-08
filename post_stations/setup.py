from setuptools import setup #,find_packages

package_name = 'post_stations'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    #find_packages(exclude=['test']),
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'post_instruction_sets'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Reusable base station logic for POST system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver_station = post_stations.receiver_station:main',
            'sender_station = post_stations.sender_station:main',
            'processing_station = post_stations.processing_station:main'
        ]
    }
)
