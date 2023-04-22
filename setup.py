from setuptools import setup

package_name = 'vcgencmd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/vcgen_monitor_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan Friedman',
    maintainer_email='ryanfriedman5410+github@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vcgen_monitor = vcgencmd.vcgen_monitor:main'
        ],
    },
)
