import os
import requests
import shutil

from ament_index_python.packages import get_package_share_directory
from urllib.parse import urlparse
from xml.etree.ElementTree import ElementTree, Element, SubElement


def get_pbr_textures(params):
    pbr_texture_types = ['metalness_map', 'roughness_map',
                         'normal_map', 'environment_map', 'light_map']
    pbr_textures = {}
    for pbr_key, param in params.items():
        if pbr_key in pbr_texture_types:
            pbr_textures[pbr_key] = param.value
    return pbr_textures


def get_ceiling_pbr_textures(params):
    ceiling_pbr_texture_types = {'ceiling_metalness_map': 'metalness_map',
                                 'ceiling_roughness_map': 'roughness_map',
                                 'ceiling_environment_map': 'environment_map',
                                 'ceiling_normal_map': 'normal_map',
                                 'ceiling_light_map': 'light_map',
                                 'ceiling_emissive_map': 'emissive_map'}
    ceiling_pbr_textures = {}
    for ceiling_pbr_key, param in params.items():
        if ceiling_pbr_key in ceiling_pbr_texture_types:
            pbr_key = ceiling_pbr_texture_types[ceiling_pbr_key]
            ceiling_pbr_textures[pbr_key] = param.value
    return ceiling_pbr_textures


def copy_texture(texture_name, dest_path):
    texture_filename = f'{texture_name}.png'
    texture_path_dest = f'{dest_path}/{texture_filename}'
    # Create the destination path if it does not exist
    if not os.path.exists(dest_path):
        os.makedirs(dest_path)
    # If the texture name is a URL fetch it
    result = urlparse(texture_name)
    texture_is_url = all([result.scheme, result.netloc, result.path])
    if texture_is_url:
        # Update filename from the URL
        texture_filename = texture_name.split('/')[-1]
        texture_path_dest = f'{dest_path}/{texture_filename}'
        # Skip downloading if existing
        if not os.path.isfile(texture_path_dest):
            req = requests.get(texture_name)
            with open(texture_path_dest, 'wb') as f:
                f.write(req.content)
    else:
        texture_path_source = os.path.join(
            get_package_share_directory('rmf_building_map_tools'),
            f'textures/{texture_filename}')
        shutil.copyfile(texture_path_source, texture_path_dest)

    print(f'  wrote {texture_path_dest}')
    return texture_filename


def add_pbr_material(visual_ele, model_name, obj_name,
                     texture_name, meshes_path, pbr_textures):
    # Check if the texture is a URL and copy texture
    texture_filename = copy_texture(texture_name, meshes_path)
    material_ele = SubElement(visual_ele, 'material')
    diffuse_ele = SubElement(material_ele, 'diffuse')
    diffuse_ele.text = '1.0 1.0 1.0'
    specular_ele = SubElement(material_ele, 'specular')
    specular_ele.text = '0.1 0.1 0.1'  # TODO check specular value
    pbr_ele = SubElement(material_ele, 'pbr')
    metal_ele = SubElement(pbr_ele, 'metal')
    metalness_ele = SubElement(metal_ele, 'metalness')
    # Sensible default for floors / walls not to be made out of metal
    # TODO parametrize
    metalness_ele.text = '0.0'
    # Diffuse tag for PBR
    albedo_map_ele = SubElement(metal_ele, 'albedo_map')
    albedo_map_ele.text = f'model://{model_name}/meshes/{texture_filename}'
    # Now add all PBR textures
    pbr_map_eles = []
    for pbr_type, pbr_name in pbr_textures.items():
        pbr_map_eles.append(SubElement(metal_ele, pbr_type))
        if (pbr_type == 'light_map'):
            pbr_map_eles[-1].set('uv_set', '1')
        pbr_filename = copy_texture(pbr_name, meshes_path)
        pbr_map_eles[-1].text = f'model://{model_name}/meshes/{pbr_filename}'

    # For compatibility, create material script with diffuse texture for gazebo
    script_ele = SubElement(material_ele, 'script')
    script_uri_ele = SubElement(script_ele, 'uri')
    script_uri_ele.text = f'model://{model_name}/meshes/'
    material_name = f'{obj_name}_Diffuse'
    script_name_ele = SubElement(script_ele, 'name')
    script_name_ele.text = material_name
    # Now create the OGRE1 script, append diffuse material
    with open(f'{meshes_path}/model.material', 'a') as f:
        f.write(f'material {material_name}\n')
        f.write('{\n')
        f.write('\ttechnique\n')
        f.write('\t{\n')
        f.write('\t\tpass\n')
        f.write('\t\t{\n')
        f.write('\t\t\ttexture_unit\n')
        f.write('\t\t\t{\n')
        f.write(f'\t\t\t\ttexture {texture_filename}\n')
        f.write('\t\t\t}\n')
        f.write('\t\t}\n')
        f.write('\t}\n')
        f.write('}\n')
    # TODO remove
    return texture_filename
