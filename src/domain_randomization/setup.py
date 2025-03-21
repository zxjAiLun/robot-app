from setuptools import setup
import os
from glob import glob

package_name = 'domain_randomization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Domain randomization package for robotic simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'domain_randomizer = domain_randomization.domain_randomizer:main',
            'material_randomizer = domain_randomization.material_randomizer:main',
            'dynamics_randomizer = domain_randomization.dynamics_randomizer:main',
            'randomization_manager = domain_randomization.main:main',
        ],
    },
)
