from setuptools import find_packages, setup

package_name = 'docking_assist_sensor'

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
    maintainer='lolddohaja',
    maintainer_email='progryu@zetabank.co.kr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_guide = docking_assist_sensor.docking_guide:main'
            'docking_sensor = docking_assist_sensor.docking_sensor:main'
        ],
    },
)
