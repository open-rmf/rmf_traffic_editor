#!/usr/bin/env python3
import sys
import generator


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('usage: nav_generator INPUT.yaml OUTPUT_PREFIX')
        sys.exit(1)
    input_filename = sys.argv[1]
    output_prefix = sys.argv[2]
    g = generator.Generator()
    g.generate_nav(input_filename, output_prefix)
