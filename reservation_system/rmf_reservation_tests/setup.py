from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

package_name = 'rmf_reservation_tests'

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
    maintainer='Arjoc',
    maintainer_email='arjoc@google.com',
    description='Launch tests for RMF reservation system.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
