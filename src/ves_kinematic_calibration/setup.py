from setuptools import setup

package_name = 'ves_kinematic_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Python node for kinematic calibration of VES robot using NDI and ATI sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           
        
           
            'ves_joint_publisher = ves_kinematic_calibration.ves_joint_publisher:main',
          
            'ves_cp_publisher_smooth = ves_kinematic_calibration.ves_cp_publisher_smooth:main',
            
            'palpation_grid = ves_kinematic_calibration.palpation_grid:main',

        ],
    },
)
