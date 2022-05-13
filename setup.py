import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rtk_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Angus Stewart',
    author_email='siliconlad@protonmail.com',
    maintainer='Angus Stewart',
    maintainer_email='siliconlad@protonmail.com',
    description='A utility to map the cones of a Formula Student track using an RTK receiver.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapper = rtk_mapper.mapper:main',
        ],
    },
)
