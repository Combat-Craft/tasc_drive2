from setuptools import find_packages, setup

package_name = 'drive_control'

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
    maintainer='ahmedtabl',
    maintainer_email='ahmedmegahed20142@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_position_publisher = drive_control.motor_position_publisher:main',
            'jetson_relay = drive_control.jetson_relay:main',
        ],
    },
)
