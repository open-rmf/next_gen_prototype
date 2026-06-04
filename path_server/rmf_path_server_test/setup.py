from setuptools import find_packages
from setuptools import setup

package_name = 'rmf_path_server_test'

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
    maintainer='Arjo Chakravarty',
    maintainer_email='arjoc@intrinsic.ai',
    description='Launch tests for RMF path server scenario.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rmf_visualize_trajectory = rmf_path_server_test.visualize:main',
        ],
    },
)
