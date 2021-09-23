#!/usr/bin/env python3

from building_map.generator import Generator

try:
    from building_map_generator._init_argparse import parser
except Exception:
    from _init_argparse import parser


def main():
    args = parser.parse_args()
    g = Generator()

    if args.command == "gazebo":
        g.generate_gazebo_sdf(
            args.INPUT,
            args.OUTPUT_WORLD,
            args.OUTPUT_MODEL_DIR,
            args.options
        )

    if args.command == "ignition":
        g.generate_ignition_sdf(
            args.INPUT,
            args.OUTPUT_WORLD,
            args.OUTPUT_MODEL_DIR,
            args.options
        )

    if args.command == "ignition_dae_export":
        g.generate_ignition_sdf_with_baked_worlds(
            args.INPUT,
            args.OUTPUT_WORLD_DIR,
            args.BAKED_WORLD_FILE,
            args.OUTPUT_MODEL_DIR
        )

    if args.command == "nav":
        g.generate_nav(args.INPUT, args.OUTPUT_DIR)


if __name__ == "__main__":
    main()
