from setuptools import setup

package_name = 'my_package_py'

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
    maintainer='mi',
    maintainer_email='mi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node_py = my_package_py.my_node_py:main',
            'save_image = my_package_py.save_image:main',
            'red_ball_detect = my_package_py.red_ball_detect:main',
            'ultrasonic_listener = my_package_py.ultrasonic_listener:main',
            'tof_listener = my_package_py.tof_listener:main'
        ],
    },
)
