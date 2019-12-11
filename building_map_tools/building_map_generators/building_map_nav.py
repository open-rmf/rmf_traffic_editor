import sys
from building_map.generator import Generator


def main():
    if len(sys.argv) != 3:
        print('usage: building_map_nav INPUT.yaml OUTPUT_DIRECTORY')
        sys.exit(1)
    input_filename = sys.argv[1]
    output_dir = sys.argv[2]
    g = Generator()
    g.generate_nav(input_filename, output_dir)
