from setuptools import setup
import os
from glob import glob

package_name = 'bot_bestie'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # This line ensures the package is discoverable
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('bot_bestie/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongyi',
    maintainer_email='hongyilin.mail@gmail.com',
    description='ROS 2 bot_bestie navigation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_controller = bot_bestie.nodes.global_controller:main',
            'navigation_node = bot_bestie.nodes.navigation_node:main'
        ],
    },
)
