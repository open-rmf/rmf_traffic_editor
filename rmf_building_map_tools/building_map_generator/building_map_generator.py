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
        g.generate_sdf(
            args.INPUT,
            args.OUTPUT_WORLD,
            args.OUTPUT_MODEL_DIR,
            args.TEMPLATE_WORLD_FILE,
            args.SKIP_CAMERA_POSE
        )

    if args.command == "nav":
        g.generate_nav(args.INPUT, args.OUTPUT_DIR)

    if args.command == "navgraph_visualization":
        g.generate_navgraph_visualization(args.INPUT, args.OUTPUT_DIR)


if __name__ == "__main__":
    main()
