from setuptools import setup

package_name = 'lane_follow_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Lane follow node (lane detection + steering)',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follow_node = lane_follow_pkg.lane_follow_node:main',
        ],
    },
)
