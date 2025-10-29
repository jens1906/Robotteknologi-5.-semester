from setuptools import find_packages, setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='A ROS2 package with Python nodes for robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_robot_package.publisher_node:main',
            'subscriber_node = my_robot_package.subscriber_node:main',
        ],
    },
)
