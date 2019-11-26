import sys
import building_map


def main():
    if len(sys.argv) != 3:
        print('usage: building_map_gazebo INPUT.yaml OUTPUT.world')
        sys.exit(1)
    input_filename = sys.argv[1]
    output_filename = sys.argv[2]
    g = building_map.Generator()
    g.generate_sdf(input_filename, output_filename)
