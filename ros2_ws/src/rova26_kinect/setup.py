from setuptools import setup

package_name = 'rova26_kinect'

setup(
    name='rova26_kinect',
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'rova26_kinect_publisher = rova26_kinect.rova26_kinect_publisher:main',
            'rova26_map_subscriber = rova26_kinect.rova26_map_subscriber:main',
        ],
    },
)
