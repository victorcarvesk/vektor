from setuptools import setup
import os
from glob import glob

package_name = 'vektor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'),
            glob(os.path.join('description', '*.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor Carvesk',
    maintainer_email='victorcarvesk@gmail.com',
    description='license test',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vektor_node = vektor_pkg.vektor_node:main'
        ],
    },
)
