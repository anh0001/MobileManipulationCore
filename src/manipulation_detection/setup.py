from setuptools import setup
import os
from glob import glob

package_name = 'manipulation_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MobileManipulationCore Team',
    maintainer_email='robo@example.com',
    description='Remote detection bridge and Grounding DINO server for visual servo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'remote_detection_client = manipulation_detection.remote_detection_client:main',
            'detection_server = manipulation_detection.detection_server:main',
            'detection_prompt_cli = manipulation_detection.detection_prompt_cli:main',
        ],
    },
)
