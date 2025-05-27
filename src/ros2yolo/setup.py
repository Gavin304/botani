from setuptools import find_packages, setup

package_name = 'ros2yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'ros2yolo': ['prompt.txt'],  # Include prompt.txt in the package
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gavin',
    maintainer_email='gavin.arpandy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2yolo = ros2yolo.ros2yolo:main'
        ],
    },
)
