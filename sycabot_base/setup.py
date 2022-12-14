from setuptools import setup

package_name = 'sycabot_base'

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
    maintainer='sycamore',
    maintainer_email='pierre.chassagne@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = sycabot_base.motors_waveshare:main',
            'advertise_id = sycabot_base.beaconClient:main'
        ],
    },
)
