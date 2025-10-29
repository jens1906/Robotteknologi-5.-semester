from setuptools import setup

package_name = 'vision_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    scripts=['scripts/vision_processor'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='Subscribes to camera data and processes it',
    license='MIT',
)
