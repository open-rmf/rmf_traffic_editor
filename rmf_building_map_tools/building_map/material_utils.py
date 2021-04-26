import os
import requests
import shutil

from ament_index_python.packages import get_package_share_directory
from urllib.parse import urlparse
from xml.etree.ElementTree import ElementTree, Element, SubElement


def copy_texture(texture_name, dest_path):
    texture_filename = f'{texture_name}.png'
    texture_path_dest = f'{dest_path}/{texture_filename}'
    # If the texture name is a URL fetch it
    result = urlparse(texture_name)
    texture_is_url = all([result.scheme, result.netloc, result.path])
    if texture_is_url:
        req = requests.get(texture_name)
        # Update filename from the URL
        texture_filename = texture_name.split('/')[-1]
        texture_path_dest = f'{dest_path}/{texture_filename}'
        with open(texture_path_dest, 'wb') as f:
            f.write(req.content)
    else:
        texture_path_source = os.path.join(
            get_package_share_directory('rmf_building_map_tools'),
            f'textures/{texture_filename}')
        shutil.copyfile(texture_path_source, texture_path_dest)

    print(f'  wrote {texture_path_dest}')
    return texture_filename
