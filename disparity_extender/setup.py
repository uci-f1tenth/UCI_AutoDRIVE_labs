from setuptools import find_packages, setup

package_name = 'disparity_extender'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/disparity_extender.launch.py']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alistair Keiller',
    maintainer_email='alistair@keiller.net',
    description='A ROS disparity extender implementation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'disparity_extender = disparity_extender.disparity_extender:main',
        ],
    },
)
