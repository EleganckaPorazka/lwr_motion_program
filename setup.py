import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lwr_motion_program'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labrob',
    maintainer_email='lukasz.wolinski@pw.edu.pl',
    description='A package to program a motion of LWR 4+ manipulator.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ptp_demo = lwr_motion_program.ptp_demo:main',
        'lin_demo = lwr_motion_program.lin_demo:main'
        ],
    },
)
