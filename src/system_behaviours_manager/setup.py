import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'system_behaviours_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leeros',
    maintainer_email='laluk321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'system_state_mocker = {package_name}.system_state_mocker_node:main',
            f'system_behaviour_tree = {package_name}.system_behaviour_node:main',
            f'ros_behaviour_tree = {package_name}.ros_behaviour_tree:main',
        ],
    },
)
