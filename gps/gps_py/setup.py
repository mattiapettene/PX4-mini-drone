from setuptools import setup

package_name = 'gps_py'

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
    maintainer='mattia',
    maintainer_email='81318801+mattiapettene@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_position_py = gps_py.point_list:main',
            'gps_velocity_py = gps_py.vel_ctrl:main',
        ],
    },
)