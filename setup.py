from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'emc2_pyserial_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/emc2_arduino_serial_comm.py']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'test_drive_client = emc2_pyserial_test.test_drive_client:main',
            'test_drive_server = emc2_pyserial_test.test_drive_server:main',
            'emc2_arduino_serial_comm = emc2_pyserial_test.emc2_arduino_serial_comm:main',
        ],
    },
)
