#!/usr/bin/env python3

from building_map.generator import Generator
from building_map_generator._init_argparse import parser

import pit_crew
import logging
import sys

handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(pit_crew.PitCrewFormatter())
logger = logging.getLogger()
logger.addHandler(handler)
logger.setLevel(logging.INFO)


def main():
    args = parser.parse_args()
    g = Generator()

    if args.command == "gazebo":
        # Construct model set
        model_set = set()

        if not args.no_download:
            building = g.parse_editor_yaml(args.INPUT)

            for _, level in building.levels.items():
                for model in level.models:
                    model_set.add(model.model_name)

        missing_models = pit_crew.get_missing_models(
            model_set,
            model_path=args.model_path,
            cache_file_path=args.cache
        )

        for downloadable_model in missing_models['downloadable']:
            model_name, author_name = downloadable_model

            # TODO: For now it just picks the first author
            # When model authors are put into the thumbnails, then we can
            # use that instead
            logger.info("\nDownloading %s by %s from Fuel"
                        % (model_name, author_name[0]))

            pit_crew.download_model(model_name, author_name[0],
                                    download_path=args.model_path)

        logger.warning("\nUnavailable models (not in local or Fuel): %s"
                       % missing_models['missing'])

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

    if args.command == "nav":
        g.generate_nav(args.INPUT, args.OUTPUT_DIR)


if __name__ == "__main__":
    main()
