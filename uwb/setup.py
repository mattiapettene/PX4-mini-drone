from setuptools import setup

package_name = 'uwb'

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
            'uwb = uwb.uwb:main',
            'read_sensor = uwb.read_sensor:main',
            'bridge_uwb = uwb.bridge_uwb_sitl:main',
        ],
    },
)
