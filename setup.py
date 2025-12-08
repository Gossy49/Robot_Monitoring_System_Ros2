from setuptools import find_packages, setup

package_name = 'lab2_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system_launch.py']),
        ('share/' + package_name + '/config', ['config/state_monitor.yaml', 'config/error_handler.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='godstimeokolo',
    maintainer_email='godstimeokolo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor1 = lab2_py_pkg.sensor1:main',
            'sensor2 = lab2_py_pkg.sensor2:main',
            'state_monitor = lab2_py_pkg.state_monitor:main',
            'error_handler = lab2_py_pkg.error_handler:main'
        ],
    },
)
