from setuptools import setup

package_name = 'berry_to_elfin'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_to_elfin.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco',
    maintainer_email='you@example.com',
    description='Bridge YOLO xyz -> Elfin MoveIt',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_to_elfin = berry_to_elfin.yolo_to_elfin:main',
        ],
    },
)
