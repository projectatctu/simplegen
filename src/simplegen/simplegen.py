#!/usr/bin/env python3

import argparse

import rospy
from .example_maps import AVAILABLE_MAPS


def parse_args():
    parser = argparse.ArgumentParser(description="simplegen")
    parser.add_argument("--map", type=str, help="Map to generate", choices=list(AVAILABLE_MAPS.keys()))
    parser.add_argument("--visualize", action="store_true", help="Visualize the map in rviz")
    parser.add_argument("--roll", type=float, help="Roll of the box", default=0.0)
    parser.add_argument("--pitch", type=float, help="Pitch of the box", default=0.0)
    parser.add_argument("--yaw", type=float, help="Yaw of the box", default=0.0)   
    return parser.parse_args()


def main():
    args = parse_args()
    print(args)

    if args.map == "TestMap":
        map = AVAILABLE_MAPS[args.map](args.roll, args.pitch, args.yaw).setup()
    else:
        map = AVAILABLE_MAPS[args.map]().setup()

    if args.visualize:
        rospy.init_node("simplegen_rviz_visualizer")
        map.rviz_visualize()
        rospy.spin()

    map.save_map("test.world")

    print("Bye!")


if __name__ == "__main__":
    main()
