from setuptools import find_packages, setup

package_name = 'cosmo'

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
    maintainer='Youma Leng-Nijo',
    maintainer_email='yln28@bath.ac.uk',
    description='ROS2 Implementation for the COSMO rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "control_node = cosmo.control_node:main",
            "flask_node = cosmo.flask_node:main",
            "model_node = cosmo.model_node:main",
            "motor_driver_node = cosmo.motor_driver_node:main",

        ],
    },
)
