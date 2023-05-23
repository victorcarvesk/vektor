from setuptools import setup
import os
from glob import glob

package_name = 'vektor_pkg'

package_folders = []
package_folders.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
package_folders.append(('share/' + package_name, ['package.xml']))
package_folders.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
package_folders.append((os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))))
package_folders.append((os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.xacro'))))
package_folders.append((os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_folders,
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
