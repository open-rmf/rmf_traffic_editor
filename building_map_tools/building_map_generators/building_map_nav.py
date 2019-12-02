import sys
import building_map


def main():
    if len(sys.argv) != 3:
        print('usage: building_map_nav INPUT.yaml OUTPUT_PREFIX')
        sys.exit(1)
    input_filename = sys.argv[1]
    output_prefix = sys.argv[2]
    g = building_map.Generator()
    g.generate_nav(input_filename, output_prefix)
