from setuptools import find_packages, setup

package_name = 'bot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/launch', ['launch/amcl.launch.py']),
        ('share/' + package_name + '/config', ['config/amcl.yaml']),
        ('share/' + package_name + '/maps', ['maps/map.yaml']),
        ('share/' + package_name + '/maps', ['maps/map.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agv',
    maintainer_email='agv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apriltag_to_pose_node = bot_localization.apriltag_to_pose_node:main'
        ],
    },
)
