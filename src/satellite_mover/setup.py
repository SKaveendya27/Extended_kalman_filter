import os
from setuptools import find_packages, setup
from glob import glob
package_name = 'satellite_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/world', glob('world/*.xml')),
        ('share/'+ package_name + '/launch', glob('launch/*.py')),
        ('share/'+ package_name + '/config', glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='supuni',
    maintainer_email='skaveendya27@gmail.com',
    description='Package to control the satellite in microgravity world',
    license='TODO: License declaration',
    extras_require={  
        'test': [
            'pytest',
            'ament_copyright',
            'ament_flake8',
            'ament_pep257',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_satellite = satellite_mover.move_satellite:main',
            'sat_pose_ekf_plot = satellite_mover.sate_pose_ekf_plot:main',
            'OdometryNoiseAdder = satellite_mover.OdometryNoiseAdder:main',
            'VisualOdometry_publisher = satellite_mover.VisualOdometry_publisher:main',
            'DummyPublisher = satellite_mover.dummy_array_publisher:main',
            'odom_plotter = satellite_mover.odom_plotter:main'
        ],
    },
)

