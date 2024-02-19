#!/usr/bin/env python3

import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="simplegen")
    return parser.parse_args()

def main():
    parse_args()
    print("Bye!")

if __name__ == "__main__":
    main()