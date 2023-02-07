from setuptools import setup

package_name = 'sensor_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Liang',
    maintainer_email='liang.y.andrew@gmail.com',
    description='3-DOF Sensor wrapper and service package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = sensor_service.server:main',
            'client = sensor_service.client:main',
        ],
    },
)
