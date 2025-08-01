from setuptools import find_packages, setup
from glob import glob
package_name = 'box_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/worlds', glob('worlds/*.world')),
        ('share/'+ package_name + '/launch', glob('launch/*.py')),
        ('share/'+ package_name + '/config', glob('config/*.yaml')),
        ('share/'+ package_name + '/models', glob('models/*.sdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='supuni',
    maintainer_email='supuni@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_box = box_mover.move_box:main',
        ],
    },
)
