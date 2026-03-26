from setuptools import setup
import os
from glob import glob

package_name = 'ves_kinematic_calibration'

setup(
    name=package_name,
    version='0.1.0',
    # This finds the subfolder with the same name containing your .py files
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If you have launch files, this includes them:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yufei',
    maintainer_email='yufei@example.com',
    description='Python node for kinematic calibration of VES robot using NDI and ATI sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ves_joint_publisher = ves_kinematic_calibration.ves_joint_publisher:main',
            'ves_cp_publisher_smooth = ves_kinematic_calibration.ves_cp_publisher_smooth:main',
            'palpation_grid = ves_kinematic_calibration.palpation_grid:main',
            'palpation_kuka = ves_kinematic_calibration.palpation_kuka:main',
            'palpation_hover = ves_kinematic_calibration.palpation_hover:main',
            'palpation_nohome = ves_kinematic_calibration.palpation_nohome:main',
            'cp_calibration = ves_kinematic_calibration.cp_calibration:main',
            'jp_calibration = ves_kinematic_calibration.jp_calibration:main',
            'force_cp = ves_kinematic_calibration.force_cp:main',
            'ves_ik_publisher = ves_kinematic_calibration.ves_ik_publisher:main',
        ],
    },
)