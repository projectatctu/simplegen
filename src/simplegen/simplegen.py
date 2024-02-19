#!/usr/bin/env python3

import argparse

from .example_maps import AVAILABLE_MAPS


def parse_args():
    parser = argparse.ArgumentParser(description="simplegen")
    parser.add_argument(
        "--map", type=str, help="Map to generate", choices=list(AVAILABLE_MAPS.keys()) + [""], default=""
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print(args)

    if args.map != "":
        map = AVAILABLE_MAPS[args.map]().setup()
        print(map)
        map.rviz_visualize()

    print("Bye!")


if __name__ == "__main__":
    main()
