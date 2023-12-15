from setuptools import find_packages, setup

package_name = 'core_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristanmcginnis26@gmail.com',
    description='Core rover control package to handle command interpretation and embedded interfacing.',
    license='All Rights Reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "core_control = core_pkg.core_control_node:main"
        ],
    },
)
