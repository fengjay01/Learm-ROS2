from setuptools import find_packages, setup

package_name = 'scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='china',
    maintainer_email='china@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'print_trajectory = scripts.print_trajectory:main',
            'trajectory_relay = scripts.trajectory_relay:main',
            'moveit_stm32 = scripts.moveit_stm32:main',
        ],
    },
)
