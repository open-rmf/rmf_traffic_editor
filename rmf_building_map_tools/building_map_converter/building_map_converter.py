#!/usr/bin/env python3
import argparse
import os
import yaml

from building_map.building import Building


def main():
    parser = argparse.ArgumentParser(
        prog='building_map_converter',
        description='Combine .building.yaml files to/from other formats'
    )

    parser.add_argument('INPUT_YAML', type=str,
                        help='building.yaml filename')
    parser.add_argument('OUTPUT', type=str,
                        help='output filename')

    args = parser.parse_args()

    if not os.path.isfile(args.INPUT_YAML):
        raise FileNotFoundError(f'input file {args.INPUT_YAML} not found')

    with open(args.INPUT_YAML, 'r') as f:
        y = yaml.load(f, Loader=yaml.CLoader)

    b = Building(y)

    if args.OUTPUT.endswith('.gpkg'):
        b.generate_geopackage_file(args.OUTPUT)
    elif args.OUTPUT.endswith('.geojson'):
        b.generate_geojson_file(args.OUTPUT)
    elif args.OUTPUT.endswith('.geojson.gz'):
        b.generate_geojson_file(args.OUTPUT, True)
    else:
        raise ValueError(f'unknown filename suffix: {args.OUTPUT}')


if __name__ == '__main__':
    main()
