from setuptools import setup

package_name = 'rova26'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattthewhumble',
    maintainer_email='mattthewhumble@todo.todo',
    description='ROS2 mapping node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_received_node = rova26.map_received_node:main',
        ],
    },
)
