from setuptools import find_packages, setup

package_name = 'bot_apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/config', ['config/tag_map.yaml']),
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
            'apriltag_detector = bot_apriltag.apriltag_detector:main',
            'apriltag_localizer = bot_apriltag.apriltag_localizer:main',
            'apriltag_relocator = bot_apriltag.apriltag_relocalizer:main',
        ],
    },
)
