from setuptools import find_packages, setup

package_name = 'pkg1'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ezekiel A. Mitchell',
    maintainer_email='ezekiel@endr.us',
    description='Introductory package to ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "python_node = pkg1.node1:main"
        ],
    },
)
