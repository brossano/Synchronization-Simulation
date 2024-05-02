from setuptools import setup
import os
from glob import glob

package_name = 'image_proc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='brossano@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync = image_proc.image_subscriber:main',
            'amr_nav = image_proc.amr_nav:main',
            'image_pub = image_proc.image_publisher:main',
            'manipulator_ros = image_proc.manipulator_node:main',
            'ultrasonic_ros = image_proc.ultrasonic_node:main'
        ],
    },
)
