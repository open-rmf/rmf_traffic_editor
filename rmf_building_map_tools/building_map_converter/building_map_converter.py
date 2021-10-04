#!/usr/bin/env python3
import argparse
import os
import yaml

from building_map.building import Building


def main():
    parser = argparse.ArgumentParser(
        prog='building_map_converter',
        description='Combine .building.yaml files to/from GeoPackage'
    )

    parser.add_argument('INPUT_YAML', type=str,
                        help='building.yaml filename')
    parser.add_argument('OUTPUT_GPKG', type=str,
                        help='GeoPackage filename')

    args = parser.parse_args()

    if not os.path.isfile(args.INPUT_YAML):
        raise FileNotFoundError(f'input file {args.INPUT_YAML} not found')

    with open(args.INPUT_YAML, 'r') as f:
        y = yaml.load(f, Loader=yaml.CLoader)

    b = Building(y)
    b.generate_geopackage_file(args.OUTPUT_GPKG)


if __name__ == '__main__':
    main()
