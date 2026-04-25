from setuptools import find_packages, setup

package_name = 'slam_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='SLAM node with occupancy grid',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_node = slam_robot.slam_node:main',
        ],
    },
)