from setuptools import find_packages, setup

package_name = 'shell_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/shell_simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sadiq',
    maintainer_email='sadiqsaidu1221@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = shell_simulation.perception_node:main'
            'planning_node = shell_simulation.planning_node:main'
            'control_node = shell_simulation.control_node:main'
        ],
    },
)
