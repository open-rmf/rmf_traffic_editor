from glob import glob
from setuptools import setup

package_name = 'building_map_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'building_crowdsim',
        'building_crowdsim.navmesh',
        'building_crowdsim.config',
        'building_map',
        'building_map.doors',
        'building_map_server',
        'building_map_generator',
        'building_map_model_downloader',
        'model_downloader',
        'pit_crew'],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/templates',
            glob('building_map/templates/*')
        ),
        (
            'share/' + package_name + '/textures',
            glob('building_map_generator/textures/*.png')
        ),
    ],
    install_requires=['setuptools', 'shapely', 'pyyaml'],
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
            'building_crowdsim = '
            'building_crowdsim.building_crowdsim:main',
            'building_map_server = '
            'building_map_server.building_map_server:main',
            'building_map_generator = '
            'building_map_generator.building_map_generator:main',
            'building_map_model_downloader = '
            'building_map_model_downloader.building_map_model_downloader:main',
            'model_downloader = '
            'model_downloader.model_downloader:main',
        ],
    },
)
