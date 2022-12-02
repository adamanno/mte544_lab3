import os
from glob import glob
from setuptools import setup

package_name = 'mte544_lab3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hari',
    maintainer_email='kaurisanselva@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test = mte544_lab3.test:main',
        'astartest = mte544_lab3.a_star_skeleton1:main',
        'astar = mte544_lab3.a_star:main',
        'move = mte544_lab3.MovementController:main',
        'acm = mte544_lab3.a_star_costmap:main',
        ],
    },
)
