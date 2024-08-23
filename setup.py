from setuptools import find_packages, setup

package_name = 'bridge_local_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sunglok Choi',
    maintainer_email='sunglok@seoultech.ac.kr',
    description='A collection of local mappers and planners for NRF-Bridge project',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_mapper_node = bridge_local_planner.local_mapper_node:main',
        ],
    },
)
