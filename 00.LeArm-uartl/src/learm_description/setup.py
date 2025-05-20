from setuptools import find_packages, setup
import os
import glob

package_name = 'learm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # # 方法一
        # (os.path.join('share', package_name, 'launch'),
        #  list(glob.glob('launch/*.py'))),
        # (os.path.join('share', package_name, 'urdf'), list(glob.glob('urdf/*.urdf'))),
        # (os.path.join('share', package_name, 'meshes'),
        #  list(glob.glob('meshes/*.STL'))),
        # (os.path.join('share', package_name, 'rviz'), list(glob.glob('rviz/*.rviz'))),
        # (os.path.join('share', package_name, 'worlds'),
        #  list(glob.glob('worlds/*.sdf'))),

        # 方法2【匹配文件（避免匹配到子目录）】
        (os.path.join('share', package_name, 'urdf'),
         glob.glob('urdf/**/*', recursive=True)),
        (os.path.join('share', package_name, 'meshes'),
         glob.glob('meshes/**/*', recursive=True)),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'),
         glob.glob('config/**/*', recursive=True)),
        (os.path.join('share', package_name, 'rviz'),
         glob.glob('rviz/**/*', recursive=True)),


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

        ],
    },
)
