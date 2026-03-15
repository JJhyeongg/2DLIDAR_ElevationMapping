from setuptools import setup

package_name = 'bunker_util'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'scripts.waypoint_follower.py'
    ],
    install_requires=['setuptools', 'nav2_simple_commander'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Example package to navigate a robot using Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_follower = scripts.waypoint_follower.py:main'
        ],
    },
)
