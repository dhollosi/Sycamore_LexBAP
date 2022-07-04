from setuptools import setup
from glob import glob

package_name = 'sycamore'

scripts = {
    'lexBAP': ['guidance', 'table', 'planner', 'controller', 'rviz', 'rviz_task','analysis'],
    'linSAP': ['guidance', 'table', 'planner', 'controller', 'rviz', 'rviz_task', 'analysis'],

}

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('resource/*.rviz')),
        ('share/' + package_name, glob('resource/*.sdf')),
        ('share/' + package_name, glob('resource/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dimitri',
    maintainer_email='dimitri.hollosi@epfl.ch',
    description='Sycamore - Task Assignment contribution',
    license='Sycamore @ EPFL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
               'sycamore_{0}_{1} = sycamore.{0}.{1}:main'.format(package, file)
               for package, files in scripts.items() for file in files
           ] + [
               'sycamore_turtlebot_spawner = sycamore.turtlebot_spawner:main'
           ],
    },
)
