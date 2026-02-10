from setuptools import setup

package_name = 'arduino_bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Bridge between ROS2 and Arduino via Serial',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = arduino_bridge_pkg.arduino_bridge_node:main',
        ],
    },
)
