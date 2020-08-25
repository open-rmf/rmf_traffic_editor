import os
import sys

from .config.configfile_generator import configfile_main
from .navmesh.navmesh_generator import navmesh_main

from ._init_argparse import parser


def main():
    args = parser.parse_args()

    if args.command == 'navmesh' :
        navmesh_main(args.INPUT, args.OUTPUT_DIR, args.OUT_PREFIX)

    if args.command == 'configfile' :
        configfile_main(args.INPUT, args.OUTPUT_DIR, args.PLATFORM, args.WORLD_FILE_PROCESSED)

if __name__ == '__main__' :
    sys.exit(main())
