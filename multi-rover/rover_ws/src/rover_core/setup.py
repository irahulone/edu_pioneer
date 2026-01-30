from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'cobs'],
    zip_safe=True,
    maintainer='zen',
    maintainer_email='zen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_main = rover_core.rover_processor:main'
        ],
    },
)
