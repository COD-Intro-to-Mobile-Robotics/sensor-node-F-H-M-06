from setuptools import find_packages, setup

package_name = 'param_pubsub'

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
    maintainer='instructor',
    maintainer_email='instructor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = param_pubsub.param_publisher:main',
                'listener = param_pubsub.param_subscriber:main',
                'faraazPublisher = param_pubsub.faraaz_publisher:main'

        ],
},
)
