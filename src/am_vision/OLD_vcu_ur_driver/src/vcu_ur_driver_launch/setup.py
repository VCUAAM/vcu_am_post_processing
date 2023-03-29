from setuptools import setup

package_name = 'vcu_ur_driver_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        'launch/vcu-ur5e.launch.py',
        'etc/vcu_ur5e_calibration.yaml',
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosnuc',
    maintainer_email='rosnuc@todo.todo',
    description='Package containing calibrations and launch files for VCU UR robots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
