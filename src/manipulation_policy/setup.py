from setuptools import setup
import os
from glob import glob

package_name = 'manipulation_policy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MobileManipulationCore Team',
    maintainer_email='robo@example.com',
    description='Policy model client/inference using VLA models (OpenVLA, LeRobot)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_node = manipulation_policy.policy_node:main',
            'policy_server = manipulation_policy.policy_server:main',
        ],
    },
)
