from setuptools import find_packages, setup

package_name = 'esp32_serial'

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
    maintainer='p31h',
    maintainer_email='p31h@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_serial_node = esp32_serial.esp32_serial:main',
            'vel_test_node = esp32_serial.test_vel:main',
        ],
    },
)
