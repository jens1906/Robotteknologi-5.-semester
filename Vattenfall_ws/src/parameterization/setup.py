from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parameterization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'examples'), glob('examples/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'scikit-learn',
    ],
    zip_safe=True,
    maintainer='jens',
    maintainer_email='jens1906@gmail.com',
    description='Surface parameterization for robotic applications using inverse interpolation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'parameterization_node = parameterization.parameterization_node:main'
        ],
    },
)
