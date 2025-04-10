from setuptools import find_packages, setup

package_name = 'ai_model_node'

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
    include_package_data=True,
    zip_safe=True,
    maintainer='yln28',
    maintainer_email='yln28@bath.ac.uk',
    description='Node to house the machine learning models',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ml_node = ai_model_node.ml_node:main",
        ],
    },
)
