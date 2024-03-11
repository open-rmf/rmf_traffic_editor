import math
import os

import shapely.geometry
import shapely.ops

from xml.etree.ElementTree import SubElement

from .material_utils import (add_pbr_material, get_pbr_textures,
                             get_ceiling_pbr_textures)
from .param_value import ParamValue

triangulation_debugging = False

if triangulation_debugging:
    import numpy as np
    import matplotlib.pyplot as plt


class Floor:
    def __init__(self, yaml_node):
        self.vertex_indices = []
        self.vertices = []
        self.thickness = 0.1
        self.wall_height = 2.5
        if 'vertices' in yaml_node and yaml_node['vertices'] is not None:
            self.vertex_indices = [i for i in yaml_node['vertices']]

        self.params = {}
        if 'parameters' in yaml_node and yaml_node['parameters']:
            for param_name, param_yaml in yaml_node['parameters'].items():
                self.params[param_name] = ParamValue(param_yaml)

        self.indoor = 0
        if 'indoor' in self.params:
            self.indoor = self.params['indoor'].value

    def to_yaml(self):
        y = {}
        y['vertices'] = self.vertex_indices
        y['parameters'] = {}
        for param_name, param_value in self.params.items():
            y['parameters'][param_name] = param_value.to_yaml()
        return y

    def __str__(self):
        return f'floor ({len(self.vertices)} vertices)'

    def __repr__(self):
        return self.__str__()

    def has_ceiling(self):
        return self.indoor

    def find_vertex_idx(self, x, y, failure_ok=False):
        for v_idx, v in enumerate(self.vertices):
            dx = x - v.x
            dy = y - v.y
            d = math.sqrt(dx*dx + dy*dy)
            if d < 0.0001:
                return v_idx
        if not failure_ok:
            raise RuntimeError("Couldn't find vertex index!")
        else:
            return -1

    def add_vertex_if_needed(self, x, y):
        idx = self.find_vertex_idx(x, y, True)
        if idx >= 0:
            return  # vertex already exists
        self.vertices.append(shapely.geometry.Point(x, y))

    def triangle_to_vertex_index_list(self, triangle, vertices):
        vertex_idx_list = []
        c = triangle.exterior.coords  # save typing, make it easier to read
        vertex_idx_list.append(self.find_vertex_idx(c[0][0], c[0][1]))
        vertex_idx_list.append(self.find_vertex_idx(c[1][0], c[1][1]))
        vertex_idx_list.append(self.find_vertex_idx(c[2][0], c[2][1]))
        return vertex_idx_list

    def triangulate_polygon(self, polygon, triangles):
        # calculate Delaunay triangulation of the polygon
        convex_triangulation = shapely.ops.triangulate(polygon)

        # the Delaunay triangulation returned by Shapely will be convex,
        # but often the floor plan polygons are not, so we have to intersect
        # each triangle with the original floorplan polygon and recurse
        # if needed (i.e. if the intersection returns a poly with >3 sides)
        for triangle in convex_triangulation:
            if triangulation_debugging:
                tri_x, tri_y = triangle.exterior.coords.xy
                plt.plot(tri_x, tri_y, 'k', linewidth=1)
                plt.pause(0.1)

            # create a new geometry by intersecting this triangle with the
            # original polygon. This may create a triangle, or it may also
            # create all sorts of things: nothing, an N-sided polygon, etc.
            geom = triangle.intersection(self.polygon)

            # if the polygon intersection has no area, ignore it.
            if geom.is_empty:
                continue
            elif geom.geom_type == 'MultiLineString':
                continue
            elif geom.geom_type == 'MultiPoint':
                continue

            # now we need to actually deal with an intersection area
            if geom.geom_type == 'Polygon':
                # simplify it to remove duplicate points, which seems to
                # happen sometimes
                poly = geom.simplify(0.001, False)
                if len(poly.exterior.coords) < 4:
                    continue  # this poly disintregrated during simplification

                # ensure the winding is in canonical order (CCW)
                poly = shapely.geometry.polygon.orient(poly)

                if triangulation_debugging:
                    poly_x, poly_y = poly.exterior.coords.xy
                    plt.plot(poly_x, poly_y, 'r', linewidth=2)

                # if this is a 3-sided polygon, hooray! it's a triangle
                if len(poly.exterior.coords) == 4:
                    triangles.append(poly)
                else:
                    # we got a N-sided polygon, N > 3. We need to recurse
                    # to chop up this polygon some more.
                    self.triangulate_polygon(poly, triangles)

                    if triangulation_debugging:
                        poly_x, poly_y = poly.exterior.coords.xy
                        plt.plot(poly_x, poly_y, 'b', linewidth=4)

            elif geom.geom_type == 'MultiPolygon':
                for poly in list(geom.geoms):
                    self.triangulate_polygon(poly, triangles)

            elif geom.geom_type == 'GeometryCollection':
                # this can happen if the original triangulation needed
                # to be clipped to lie within the original floor polygon
                # for example, if a long triangle crossed a concave region
                # and you end up with >=1 polygons and >=1 points or edges.
                for item in geom.geoms:
                    if item.geom_type == 'Polygon':
                        self.triangulate_polygon(item, triangles)

            else:
                if triangulation_debugging:
                    print(f'\n\n\nFound something weird ({geom.geom_type}).')
                    print('Ignoring it:\n')
                    print(f'  {poly.wkt}')

    def generate(
        self,
        model_ele,
        floor_cnt,
        model_name,
        model_path,
        transformed_vertices,
        holes,
        lift_vert_lists
    ):
        print(f'generating floor polygon {floor_cnt} on floor {model_name}')

        vert_list = []
        self.vertices = []
        for v_idx in self.vertex_indices:
            vx, vy = transformed_vertices[v_idx].xy()
            self.vertices.append(shapely.geometry.Point(vx, vy))
            vert_list.append((vx, vy))

        if len(vert_list) < 3:
            print(f'not enough vertices for {model_name} polygon {floor_cnt}!')
            self.polygon = None
            return

        hole_vert_lists = []
        for hole in holes:
            hole_vertices = []
            for v_idx in hole.vertex_indices:
                vx, vy = transformed_vertices[v_idx].xy()
                hole_vertices.append((vx, vy))
            hole_vert_lists.append(hole_vertices)
        print(f'hole vertices: {hole_vert_lists}')
        print(f'lift vertices: {lift_vert_lists}')

        self.polygon = shapely.geometry.Polygon(vert_list)

        for hole_vert_list in hole_vert_lists:
            hole_polygon = shapely.geometry.Polygon(hole_vert_list)
            self.polygon = self.polygon.difference(hole_polygon)

        for lift_vert_list in lift_vert_lists.values():
            lift_polygon = shapely.geometry.Polygon(lift_vert_list)
            self.polygon = self.polygon.difference(lift_polygon)

        link_ele = SubElement(model_ele, 'link')
        link_ele.set('name', f'floor_{floor_cnt}')

        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', 'visual')

        visual_geometry_ele = SubElement(visual_ele, 'geometry')

        mesh_ele = SubElement(visual_geometry_ele, 'mesh')
        mesh_uri_ele = SubElement(mesh_ele, 'uri')

        meshes_path = f'{model_path}/meshes'
        if not os.path.exists(meshes_path):
            os.makedirs(meshes_path)

        obj_model_rel_path = f'meshes/floor_{floor_cnt}.obj'
        mesh_uri_ele.text = f'model://{model_name}/{obj_model_rel_path}'

        collision_ele = SubElement(link_ele, 'collision')
        collision_ele.set('name', 'collision')
        # Use the mesh as a collision element
        collision_geometry_ele = SubElement(collision_ele, 'geometry')
        collision_mesh_ele = SubElement(collision_geometry_ele, 'mesh')
        collision_mesh_uri_ele = SubElement(collision_mesh_ele, 'uri')
        collision_mesh_uri_ele.text = \
            f'model://{model_name}/{obj_model_rel_path}'

        surface_ele = SubElement(collision_ele, 'surface')
        contact_ele = SubElement(surface_ele, 'contact')
        collide_bitmask_ele = SubElement(contact_ele, 'collide_bitmask')
        collide_bitmask_ele.text = '0x01'

        triangles = []
        self.triangulate_polygon(self.polygon, triangles)

        for triangle in triangles:
            for coord in triangle.exterior.coords:
                self.add_vertex_if_needed(coord[0], coord[1])

        if triangulation_debugging:
            plt.axis('equal')
            plt.show()

        # for unknown reasons, it seems that shapely.ops.triangulate
        # doesn't return a list of vertices and triangles as indices,
        # instead you get a bunch of coordinates, so we'll re-build
        # a triangle index list now. There must be an easier way...
        tri_vertex_indices = []
        for triangle in triangles:
            tri_vertex_indices.append(
                self.triangle_to_vertex_index_list(triangle, self.vertices))
        print(tri_vertex_indices)

        texture_scale = 1.0
        if 'texture_scale' in self.params:
            texture_scale = self.params['texture_scale'].value

        obj_path = f'{model_path}/{obj_model_rel_path}'
        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib floor_{floor_cnt}.mtl\n')
            f.write(f'o floor_{floor_cnt}\n')

            # this assumes that the vertices are in "correct" (OBJ) winding
            # ordering already. todo: detect if the winding order is
            # inverted and re-wind appropriately
            # In order for the floors to be seen from below,
            # we also add another set of vertices "below" the floor thickness
            for v in self.vertices:
                f.write(f'v {v.x} {v.y} 0\n')
                f.write(f'v {v.x} {v.y} -{self.thickness}\n')

            # in the future we may have texture tiles of a different size,
            # but for now let's assume 1-meter x 1-meter tiles, so we don't
            # need to scale the texture coordinates currently.
            for v in self.vertices:
                f.write(f'vt {v.x / texture_scale} {v.y / texture_scale} 0\n')

            # our floors are always flat (for now), so normals are up or down
            f.write(f'vn 0 0 1\n')
            f.write(f'vn 0 0 -1\n')

            f.write(f'usemtl floor_{floor_cnt}\n')
            f.write('s off\n')

            for triangle in tri_vertex_indices:
                # todo... clean this up. For now, wind the triangles both ways

                f.write('f')
                for v_idx in triangle:
                    f.write(f' {2*v_idx+1}/{v_idx+1}/1')
                f.write('\n')

                # now add the triangle on the bottom-side of the floor
                f.write('f')
                for v_idx in reversed(triangle):
                    f.write(f' {2*v_idx+2}/{v_idx+1}/2')
                f.write('\n')

        print(f'  wrote {obj_path}')

        texture_name = 'blue_linoleum'
        if 'texture_name' in self.params:
            texture_name = self.params['texture_name'].value

        pbr_textures = get_pbr_textures(self.params)
        texture_filename = add_pbr_material(
                visual_ele, model_name, f'floor_{floor_cnt}', texture_name,
                f'{model_path}/meshes', pbr_textures)

        mtl_path = f'{model_path}/meshes/floor_{floor_cnt}.mtl'
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl floor_{floor_cnt}\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write('d 1.0\n')  # alpha (maybe?)
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd {texture_filename}\n')

        print(f'  wrote {mtl_path}')

    def generate_ceiling(
        self,
        model_ele,
        ceiling_cnt,
        model_name,
        model_path,
        transformed_vertices,
        holes,
        lift_vert_lists
    ):
        print(
            f'generating ceiling polygon {ceiling_cnt} on floor {model_name}')

        vert_list = []
        self.vertices = []
        for v_idx in self.vertex_indices:
            vx, vy = transformed_vertices[v_idx].xy()
            self.vertices.append(shapely.geometry.Point(vx, vy))
            vert_list.append((vx, vy))

        if len(vert_list) < 3:
            print(
                f'not enough vertices for {model_name} polygon {ceiling_cnt}!')
            self.polygon = None
            return

        hole_vert_lists = []
        for hole in holes:
            hole_vertices = []
            for v_idx in hole.vertex_indices:
                vx, vy = transformed_vertices[v_idx].xy()
                hole_vertices.append((vx, vy))
            hole_vert_lists.append(hole_vertices)
        print(f'hole vertices: {hole_vert_lists}')
        print(f'lift vertices: {lift_vert_lists}')

        self.polygon = shapely.geometry.Polygon(vert_list)

        for hole_vert_list in hole_vert_lists:
            hole_polygon = shapely.geometry.Polygon(hole_vert_list)
            self.polygon = self.polygon.difference(hole_polygon)

        for lift_vert_list in lift_vert_lists.values():
            lift_polygon = shapely.geometry.Polygon(lift_vert_list)
            self.polygon = self.polygon.difference(lift_polygon)

        link_ele = SubElement(model_ele, 'link')
        link_ele.set('name', f'ceiling_{ceiling_cnt}')

        visual_ele = SubElement(link_ele, 'visual')
        visual_ele.set('name', 'visual')

        visual_geometry_ele = SubElement(visual_ele, 'geometry')

        mesh_ele = SubElement(visual_geometry_ele, 'mesh')
        mesh_uri_ele = SubElement(mesh_ele, 'uri')

        meshes_path = f'{model_path}/meshes'
        if not os.path.exists(meshes_path):
            os.makedirs(meshes_path)

        obj_model_rel_path = f'meshes/ceiling_{ceiling_cnt}.obj'
        mesh_uri_ele.text = f'model://{model_name}/{obj_model_rel_path}'

        collision_ele = SubElement(link_ele, 'collision')
        collision_ele.set('name', 'collision')
        # Use the mesh as a collision element
        collision_geometry_ele = SubElement(collision_ele, 'geometry')
        collision_mesh_ele = SubElement(collision_geometry_ele, 'mesh')
        collision_mesh_uri_ele = SubElement(collision_mesh_ele, 'uri')
        collision_mesh_uri_ele.text = \
            f'model://{model_name}/{obj_model_rel_path}'

        surface_ele = SubElement(collision_ele, 'surface')
        contact_ele = SubElement(surface_ele, 'contact')
        collide_bitmask_ele = SubElement(contact_ele, 'collide_bitmask')
        collide_bitmask_ele.text = '0x01'

        triangles = []
        self.triangulate_polygon(self.polygon, triangles)

        for triangle in triangles:
            for coord in triangle.exterior.coords:
                self.add_vertex_if_needed(coord[0], coord[1])

        if triangulation_debugging:
            plt.axis('equal')
            plt.show()

        tri_vertex_indices = []
        for triangle in triangles:
            tri_vertex_indices.append(
                self.triangle_to_vertex_index_list(triangle, self.vertices))
        print(tri_vertex_indices)

        ceiling_scale = 1.0
        if 'ceiling_scale' in self.params:
            ceiling_scale = self.params['ceiling_scale'].value

        obj_path = f'{model_path}/{obj_model_rel_path}'
        with open(obj_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'mtllib ceiling_{ceiling_cnt}.mtl\n')
            f.write(f'o ceiling_{ceiling_cnt}\n')

            # Ceiling only visible from below
            # Invisible from above to allow bird-eye view
            for v in self.vertices:
                f.write(f'v {v.x} {v.y} {self.wall_height}\n')

            # TO DO: adjust scale texture if not square tiles
            for v in self.vertices:
                f.write(f'vt {v.x / ceiling_scale} {v.y / ceiling_scale} 0\n')

            f.write(f'vn 0 0 -1\n')

            f.write(f'usemtl ceiling_{ceiling_cnt}\n')
            f.write('s off\n')

            for triangle in tri_vertex_indices:

                f.write('f')
                for v_idx in reversed(triangle):
                    f.write(f' {v_idx+1}/{v_idx+1}/1')
                f.write('\n')

        print(f'  wrote {obj_path}')

        ceiling_texture = 'blue_linoleum'
        if 'ceiling_texture' in self.params:
            ceiling_texture = self.params['ceiling_texture'].value

        ceiling_pbr_textures = get_ceiling_pbr_textures(self.params)
        texture_filename = add_pbr_material(
                visual_ele, model_name,
                f'ceiling_{ceiling_cnt}', ceiling_texture,
                f'{model_path}/meshes', ceiling_pbr_textures)

        mtl_path = f'{model_path}/meshes/ceiling_{ceiling_cnt}.mtl'
        with open(mtl_path, 'w') as f:
            f.write('# The Great Editor v0.0.1\n')
            f.write(f'newmtl ceiling_{ceiling_cnt}\n')
            f.write('Ka 1.0 1.0 1.0\n')  # ambient
            f.write('Kd 1.0 1.0 1.0\n')  # diffuse
            f.write('Ke 0.0 0.0 0.0\n')  # emissive
            f.write('Ns 50.0\n')  # specular highlight, 0..100 (?)
            f.write('Ni 1.0\n')  # no idea what this is
            f.write('d 1.0\n')  # alpha (maybe?)
            f.write('illum 2\n')  # illumination model (enum)
            f.write(f'map_Kd {texture_filename}\n')

        print(f'  wrote {mtl_path}')
