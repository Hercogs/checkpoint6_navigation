from setuptools import setup
import os

from glob import glob

package_name = 'nav2_apps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jecuks96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_shelf_to_ship = scripts.move_shelf_to_ship:main',
        ],
    },
)
