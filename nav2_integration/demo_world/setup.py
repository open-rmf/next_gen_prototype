from setuptools import find_packages, setup

package_name = 'demo_world'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_init = demo_world.localization_init:main',
            'robot_client = demo_world.path_reporter:main',
            'path_vis = demo_world.path_vis:main'
        ],
    },
)
