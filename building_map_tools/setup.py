from setuptools import setup

package_name = 'building_map_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=['building_map', 'building_map_server', 'building_map_generators', 'tripy'],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/textures', ['building_map_generators/textures/blue_linoleum_high_contrast.png']),
    ],
    install_requires=['setuptools'],
    author='Morgan Quigley',
    author_email='morgan@osrfoundation.org',
    zip_safe=True,
    maintainer='Morgan Quigley',
    maintainer_email='morgan@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='building_map_tools',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    scripts=[],
    entry_points={
        'console_scripts': [
            'building_map_server = building_map_server.building_map_server:main',
            'building_map_gazebo = building_map_generators.building_map_gazebo:main',
            'building_map_nav = building_map_generators.building_map_nav:main'
        ],
    },
)
