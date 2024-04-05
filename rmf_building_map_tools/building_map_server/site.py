import json
import math
import os

from rmf_building_map_msgs.msg import BuildingMap
from rmf_building_map_msgs.msg import Level
from rmf_building_map_msgs.msg import Graph
from rmf_building_map_msgs.msg import GraphNode
from rmf_building_map_msgs.msg import GraphEdge
from rmf_building_map_msgs.msg import Place
from rmf_building_map_msgs.msg import AffineImage
from rmf_building_map_msgs.msg import Door
from rmf_building_map_msgs.msg import Lift
from rmf_building_map_msgs.msg import Param

DEFAULT_CABIN_DOOR_THICKNESS = 0.05
DEFAULT_CABIN_WALL_THICKNESS = 0.1
DEFAULT_CABIN_GAP = 0.01

def load_site_json(map_path):
    map_msg = BuildingMap()
    map_dir = os.path.dirname(map_path)  # for calculating image paths
    with open(map_path, 'r') as f:
        site = json.load(f)
    map_msg.name = site["properties"]["name"]
    for level_data in site["levels"].values():
        map_msg.levels.append(parse_level(map_dir, level_data))
    for lift_data in site.get("lifts", {}).values():
        lift = parse_lift(lift_data, site)
        if lift is not None:
            map_msg.lifts.append(lift)
    # TODO(luca) add navgraph named locations for rmf-web frontend
    return map_msg

def parse_angle(angle):
    if "rad" in angle:
        return angle["rad"]
    else:
        return math.pi * angle["deg"] / 180.0

def parse_anchor(anchor):
    if anchor is None:
        return None
    if "Translate2D" in anchor:
        return anchor["Translate2D"]
    else:
        return None

def parse_lift(data, site):
    lift_msg = Lift()
    props = data["properties"]
    lift_msg.name = props["name"]
    cabin = props["cabin"]["Rect"]
    lift_msg.width = cabin["width"]
    lift_msg.depth = cabin["depth"]
    front_door_thickness = 0.0
    if "front_door" in cabin:
        # TODO(luca) use custom thickness
        front_door_thickness = DEFAULT_CABIN_DOOR_THICKNESS
    center_x = -cabin["depth"] / 2.0 - DEFAULT_CABIN_WALL_THICKNESS - DEFAULT_CABIN_GAP - front_door_thickness / 2.0
    center_y = 0.0
    left_anchor = site["anchors"].get(str(props["reference_anchors"][0]))
    right_anchor = site["anchors"].get(str(props["reference_anchors"][1]))
    left_anchor = parse_anchor(left_anchor)
    right_anchor = parse_anchor(right_anchor)
    if left_anchor is None or right_anchor is None:
        # ERROR
        return None
    lift_msg.ref_yaw = math.atan2(left_anchor[1] - right_anchor[1], left_anchor[0] - right_anchor[0])
    midpoint_x = (left_anchor[0] + right_anchor[0]) / 2.0
    midpoint_y = (left_anchor[1] + right_anchor[1]) / 2.0
    lift_msg.ref_x = center_x + midpoint_x
    lift_msg.ref_y = center_y + midpoint_y
    # TODO(luca) cabin doors
    return lift_msg

def generate_wall_graph(data):
    # Start by getting all the vertices that are used
    graph = Graph()
    graph.name = "wall_graph"
    site_id_to_wall_idx = {}
    for wall in data["walls"].values():
        for anchor_id in wall["anchors"]:
            if anchor_id not in site_id_to_wall_idx:
                anchor = data["anchors"].get(str(anchor_id), None)
                anchor = parse_anchor(anchor)
                if anchor is None:
                    return None
                node = GraphNode()
                node.x = anchor[0]
                node.y = anchor[1]
                site_id_to_wall_idx[anchor_id] = len(graph.vertices)
                # We don't need other properties for wall graphs
                graph.vertices.append(node)
        edge = GraphEdge()
        edge.edge_type = GraphEdge.EDGE_TYPE_BIDIRECTIONAL
        edge.v1_idx = site_id_to_wall_idx[wall["anchors"][0]]
        edge.v2_idx = site_id_to_wall_idx[wall["anchors"][1]]
        graph.edges.append(edge)
    return graph

def parse_level(map_dir, data):
    level_msg = Level()
    level_msg.name = data["properties"]["name"]
    level_msg.elevation = data["properties"]["elevation"]

    for drawing in data.get("drawings", {}).values():
        image = AffineImage()
        props = drawing["properties"]
        image_filename = props["name"]
        if "Local" not in props["source"]:
            # Warn that non local is not supported
            continue
        image_filename = props["source"]["Local"]
        image_path = os.path.join(map_dir, image_filename)

        if os.path.exists(image_path):
            #self.get_logger().info(f'opening: {image_path}')
            with open(image_path, 'rb') as image_file:
                image.data = image_file.read()
            #self.get_logger().info(f'read {len(image.data)} byte image')
            image.name = image_filename.split('.')[0]
            image.encoding = image_filename.split('.')[-1]
            image.scale = 1.0 / props["pixels_per_meter"]
            image.x_offset = props["pose"]["trans"][0]
            image.y_offset = props["pose"]["trans"][1]
            if "yaw" in props["pose"]["rot"]:
                yaw = parse_angle(props["pose"]["rot"]["yaw"])
            level_msg.images.append(image)
            # TODO(luca) multiple drawings are not compatible with the rest of the pipeline
            # Revisit when this is not the case
            break
        else:
            pass
            #self.get_logger().error(f'unable to open image: {image_path}')

    for door in data.get("doors", {}).values():
        door_msg = Door()
        door_msg.name = door["name"]

        v1 = parse_anchor(data["anchors"].get(str(door["anchors"][0])))
        v2 = parse_anchor(data["anchors"].get(str(door["anchors"][1])))
        if v1 is None or v2 is None:
            # LOG ERROR
            continue
        # TODO(luca) Change this based on pivot side
        door_msg.v1_x = v1[0]
        door_msg.v1_y = v1[1]
        door_msg.v2_x = v2[0]
        door_msg.v2_y = v2[1]

        if "DoubleSwing" in door["kind"]:
            door_msg.door_type = door_msg.DOOR_TYPE_DOUBLE_SWING
            swing = door["kind"]["DoubleSwing"]["swing"]
            # TODO(luca) implement forward and backward
            door_msg.motion_range = parse_angle(next(iter(swing.values())))
            door_msg.motion_direction = 1 if "Backward" in swing else -1
        elif "SingleSwing" in door["kind"]:
            door_msg.door_type = door_msg.DOOR_TYPE_SINGLE_SWING
            swing = door["kind"]["SingleSwing"]["swing"]
            # TODO(luca) implement forward and backward
            door_msg.motion_range = parse_angle(next(iter(swing.values())))
            door_msg.motion_direction = 1 if "Backward" in swing else -1
        if "DoubleSliding" in door["kind"]:
            door_msg.door_type = door_msg.DOOR_TYPE_DOUBLE_SLIDING
        elif "SingleSliding" in door["kind"]:
            door_msg.door_type = door_msg.DOOR_TYPE_SINGLE_SLIDING
        else:
            # WARN not supported
            pass
        level_msg.doors.append(door_msg)

    wall_graph = generate_wall_graph(data)
    if wall_graph is not None:
        level_msg.wall_graph = wall_graph
    else:
        print("Failed generating wall graph, skipping it...")

    return level_msg
