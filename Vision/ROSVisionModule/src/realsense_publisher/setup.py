from setuptools import setup

package_name = 'realsense_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    scripts=['scripts/realsense_publisher'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='Publishes fake camera frames',
    license='MIT',
)
