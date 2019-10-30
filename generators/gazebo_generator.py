#!/usr/bin/env python3
import sys
import generator


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('usage: gazebo_generator INPUT.yaml OUTPUT.world')
        sys.exit(1)
    input_filename = sys.argv[1]
    output_filename = sys.argv[2]
    g = generator.Generator()
    g.generate_sdf(input_filename, output_filename)
