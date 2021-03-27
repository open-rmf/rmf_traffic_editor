#!/usr/bin/env python3
import argparse
import os
import yaml

from building_map.building import Building


def main():
    parser = argparse.ArgumentParser(
        prog='building_map_combiner',
        description='Combine two .building.yaml files'
    )

    parser.add_argument('PRIMARY_INPUT', type=str,
                        help='Primary building.yaml file')
    parser.add_argument('SECONDARY_INPUT', type=str,
                        help='Secondary building.yaml file')
    parser.add_argument('OUTPUT', type=str,
                        help='Output file will be PRIMARY + lanes(SECONDARY)')

    args = parser.parse_args()

    if not os.path.isfile(args.PRIMARY_INPUT):
        raise FileNotFoundError(f'input file {args.PRIMARY_INPUT} not found')
    if not os.path.isfile(args.SECONDARY_INPUT):
        raise FileNotFoundError(f'input file {args.SECONDARY_INPUT} not found')

    with open(args.PRIMARY_INPUT, 'r') as f:
        y = yaml.safe_load(f)
        primary = Building(y)

    with open(args.SECONDARY_INPUT, 'r') as f:
        y = yaml.safe_load(f)
        secondary = Building(y)

    print(f'parsed primary and secondary inputs')
    primary.add_lanes_from(secondary)
    primary.write_yaml_file(args.OUTPUT)


if __name__ == '__main__':
    main()
