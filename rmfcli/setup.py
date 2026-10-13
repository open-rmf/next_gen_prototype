from setuptools import find_packages
from setuptools import setup

setup(
    name='rmfcli',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    extras_require={
        'completion': ['argcomplete'],
        'test': [
            'pytest',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/rmfcli',
        ]),
        ('share/rmfcli', [
            'package.xml',
            'resource/package.dsv',
        ]),
        ('share/rmfcli/environment', [
            'completion/rmf-argcomplete.bash',
            'completion/rmf-argcomplete.fish',
            'completion/rmf-argcomplete.zsh',
        ]),
    ],
    package_data={'': ['py.typed']},
    zip_safe=False,
    author='Chen Bainian',
    author_email='chenbn@a-star.edu.sg',
    maintainer='Chen Bainian',
    maintainer_email='chenbn@a-star.edu.sg',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Programming Language :: Python',
    ],
    description='RMF command line tools.',
    long_description="""\
The framework provides a single command line script which can be extended with
commands and verbs.""",
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'rmf = rmfcli.cli:main',
        ],
    }
)
