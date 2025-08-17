from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'darm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')+ glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        *[
            (os.path.join('share', package_name, 'models', os.path.dirname(os.path.relpath(f, 'models'))), [f])
            for f in glob('models/**/*', recursive=True) if os.path.isfile(f)
        ],
        *[
            (os.path.join('share', package_name, 'urdf', os.path.dirname(os.path.relpath(f, 'urdf'))), [f])
            for f in glob('urdf/**/*', recursive=True) if os.path.isfile(f)
        ],
    ],  

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akshay',
    maintainer_email='akshay@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
