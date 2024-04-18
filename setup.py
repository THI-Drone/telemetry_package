from setuptools import find_packages, setup

package_name = 'telemetry_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'common_package_py'],
    zip_safe=True,
    maintainer='Mauro',
    maintainer_email='maf0115@thi.de',
    description='ROS Node with the telemetry functionality',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_node = telemetry_package.main:main',
        ],
    },
)
