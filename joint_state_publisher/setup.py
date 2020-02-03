from setuptools import find_packages
from setuptools import setup
from moya_build.common_setuptools import gen_data_files_list
from os.path import dirname, abspath

package_name = 'joint_state_publisher'
pkg_dir = pkg_dir = str(dirname(abspath(__file__)))
dirs_to_add_recursive = ['meshes', 'launch', 'param','urdf', 'rviz']
dirs_ignore = ['__pycache__']
ext_ignore = ['.pyc']

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=gen_data_files_list(package_name, pkg_dir, dirs_to_add_recursive, dirs_ignore, ext_ignore),
    install_requires=['setuptools'],
    zip_safe=True,
    author='David V. Lu!!',
    author_email='davidvlu@gmail.com',
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'A python node to publish `sensor_msgs/JointState` messages for a '
        'robot described with URDF.'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = joint_state_publisher.joint_state_publisher:main',
        ],
    },
)
