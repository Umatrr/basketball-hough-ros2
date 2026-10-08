from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'basketball_cv'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Umatrr',
    maintainer_email='umatr@users.noreply.github.com',
    description='Basketball detection using Hough circles and color checks',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = basketball_cv.detector_node:main',
            'camera_node = basketball_cv.camera_node:main',
            'display_node = basketball_cv.display_node:main',
        ],
    },
)
