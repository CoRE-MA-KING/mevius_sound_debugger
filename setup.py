from setuptools import find_packages, setup

package_name = 'mevius_sound_debugger'

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
    maintainer='ma-king',
    maintainer_email='core_ma-king@todo.todo',
    description='Sound debugger for mevius',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_debugger_node = mevius_sound_debugger.sound_debugger_node:main'
        ],
    },
)
