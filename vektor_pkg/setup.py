from setuptools import setup
import os
from glob import glob

package_name = 'vektor_pkg'

package_folders = []
package_folders.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
package_folders.append(('share/' + package_name, ['package.xml']))
# package_folders.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
# package_folders.append((os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))))
# package_folders.append((os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.xacro'))))
# package_folders.append((os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))))

folders = ['worlds', 'launch', 'description', 'config']

for folder in folders:

    files = glob(os.path.join((folder + '/**'), '*.*') , recursive=True)
    for file in files:
        package_folders.append((os.path.join('share', package_name, os.path.dirname(file)), [file]))


def package_files(data_files, directory_list):

    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

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
