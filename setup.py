from setuptools import setup

package_name = 'computer_vision'

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
    maintainer='gblake',
    maintainer_email='gaby.blake2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_identifier = computer_vision.object_identifier:main',
            'object_tracker = computer_vision.object_tracker:main',
            'object_firer = computer_vision.object_firer:main',
            'sentry = computer_vision.sentry:main',
            'object_tracker_ROS_free = computer_vision.object_tracker_ROS_free:main'

        ],
    },
)