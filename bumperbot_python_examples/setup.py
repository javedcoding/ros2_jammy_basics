from setuptools import find_packages, setup

package_name = 'bumperbot_python_examples'

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
    maintainer='javed',
    maintainer_email='javed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = bumperbot_python_examples.simple_publisher:main', #define the execution file name=under which package.which python file:which function
            'simple_qos_publisher = bumperbot_python_examples.simple_qos_publisher:main', #define the execution file name=under which package.which python file:which function
            'simple_subscriber = bumperbot_python_examples.simple_subscriber:main', #define the execution file name=under which package.which python file:which function
            'simple_qos_subscriber = bumperbot_python_examples.simple_qos_subscriber:main', #define the execution file name=under which package.which python file:which function
            'simple_lifecycle = bumperbot_python_examples.simple_lifecycle:main', #define the execution file name=under which package.which python file:which function
        ],
    },
)
