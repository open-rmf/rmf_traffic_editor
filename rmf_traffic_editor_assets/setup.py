from setuptools import setup
from glob import glob
import os

package_name = 'rmf_traffic_editor_assets'

# Create a list of all files in the assets directory
data_files = [x for x in glob("assets/**", recursive=True)
              if os.path.isfile(x)]

# And then compute appends to the data_files parameter
data_file_appends = [('share/' + os.path.dirname(x), [x]) for x in data_files]

setup(
    name=package_name,
    version='1.5.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/rmf_traffic_editor_assets',
            ['resource/assets']),
        ('share/' + package_name, ['package.xml']),
        *data_file_appends
    ],
    install_requires=['setuptools'],
    # zip_safe=True,
    author='Brandon Ong',
    author_email='brandon@osrfoundation.org',
    maintainer='Brandon Ong',
    maintainer_email='brandon@osrfoundation.org',
    keywords=['RMF', 'traffic_editor'],
    classifiers=[
        'Intended Audience :: End-Users',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: None',
        'Topic :: Simulator World Development',
    ],
    description='Assets for use with traffic_editor.',
    license='Apache License, Version 2.0',
    tests_require=[],
    entry_points={},
)
