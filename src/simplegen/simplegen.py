#!/usr/bin/env python3

import argparse

import rospy
from .example_maps import AVAILABLE_MAPS

from .map import FromFileMap
from .map_creator import RvizMapCreator


def parse_args():
    parser = argparse.ArgumentParser(description="simplegen")
    parser.add_argument(
        "--map", type=str, help="Map to generate", choices=list(AVAILABLE_MAPS.keys()) + [""], default=""
    )
    parser.add_argument("--visualize", action="store_true", help="Visualize the map in rviz")
    parser.add_argument("--store_world", action="store_true", help="Store the map in a .world file")
    parser.add_argument("--store_ply", action="store_true", help="Store the map in a .ply file")
    parser.add_argument("--world_file", type=str, help="Name of the .world file to store the map")
    parser.add_argument("--ply_file", type=str, help="Name of the .ply file to store the map")
    parser.add_argument("--from_file", type=str, help="Visualize the map from a .world file", default="")
    parser.add_argument("--creator", action="store_true", help="Run the map creator")
    return parser.parse_args()


def main():
    args = parse_args()
    print(args)

    if args.map != "":
        map = AVAILABLE_MAPS[args.map]().setup()

    if args.from_file != "":
        map = FromFileMap(filename=args.from_file, description=args.from_file).setup()

    if args.visualize:
        rospy.init_node("simplegen_rviz_visualizer")
        map.rviz_visualize()
        rospy.spin()

    if args.creator:
        rospy.init_node("simplegen_rviz_visualizer")
        creator = RvizMapCreator()
        rospy.spin()

        map = AVAILABLE_MAPS["FromMapMap"](creator.getMapGenerator(), add_floor=True).setup()

    if args.store_world:
        map.save_map(args.world_file)

    if args.store_ply:
        map.save_ply(args.ply_file)

    print("Bye!")


if __name__ == "__main__":
    main()
