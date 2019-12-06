import sys
import building_map


def main():
    if len(sys.argv) != 4:
        print('usage: building_map_gazebo INPUT_YAML OUTPUT_WORLD OUTPUT_MODEL_DIR')
        sys.exit(1)
    input_filename = sys.argv[1]
    output_filename = sys.argv[2]
    output_model_dir = sys.argv[3]
    g = building_map.Generator()
    g.generate_sdf(input_filename, output_filename, output_model_dir)
