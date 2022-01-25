from setuptools import setup
from glob import glob
import os

package_name = 'robot_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bmchale',
    maintainer_email='mchale.blake@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'commander = {package_name}.commander:main',
            f'tf_processor = {package_name}.tf_processor:main',
            f'shell = {package_name}.cli.shell:main',
            f'control_center = {package_name}.control_center:main',
        ],
    },
)
