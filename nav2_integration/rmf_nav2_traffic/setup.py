from setuptools import find_packages, setup

package_name = 'rmf_nav2_traffic'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arjo Chakravarty',
    maintainer_email='arjoc@intrinsic.ai',
    description='Python nav2_simple_commander implementation of Next Gen RMF traffic adapter for Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_traffic = rmf_nav2_traffic.main:main',
        ],
    },
)
