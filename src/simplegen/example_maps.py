import random

from .map import Map, MapGenerator
from .shapes import Box


class ExampleMap(Map):
    def __init__(self, description: str) -> None:
        """Initialize example map with a description

        Args:
            description (str): map description
        """
        super().__init__(description)

    def _add_floor(self, map: MapGenerator, visualize: bool = False) -> None:
        """Add a floor to the map

        Args:
            map (MapGenerator): map generator
            visualize (bool) = False: whether to visualize the floor or not
        """
        floor = Box("floor", *[0, 0, -0.1], *[5, 5, 0.2], visualize=visualize)
        map.add_shape(floor)


class TestMap(ExampleMap):
    def __init__(self) -> None:
        super().__init__("Test map")

    def setup_map(self) -> MapGenerator:
        map = MapGenerator()

        # Add floor
        self._add_floor(map)

        # First step
        step1 = Box("step1", *[0, 0, 0.1], *[1.5, 1.5, 0.2], visualize=True)
        map.add_shape(step1)

        # Second step - tilted box
        step2 = Box("step2", *[-1.2, 0, 0.3], *[1.5, 1.5, 0.2], *[0, 0.3, 0], visualize=True)
        map.add_shape(step2)

        # Third step
        step3 = Box("step3", *[-2.63, 0, 0.52], *[1.5, 1.5, 0.2], visualize=True)
        map.add_shape(step3)

        return map


class StairsMap(ExampleMap):
    def __init__(self, n_stairs: int = 10) -> None:
        super().__init__("Stairs")
        self.n_stairs = n_stairs

    def setup_map(self) -> MapGenerator:
        map = MapGenerator()

        # add floor
        self._add_floor(map)

        # TODO(lnotspotl): Make stair parameters configurable
        STAIRS_WIDTH = 3.0
        STAIRS_HEIGHT = 0.2
        STAIRS_DEPTH = 0.3
        X_OFFSET = 0.5

        # add stairs
        for i in range(self.n_stairs):
            add = 0.0
            if i == (self.n_stairs - 1):
                add = 1.0
            p = [X_OFFSET + i * STAIRS_DEPTH + add / 2, 0, i * STAIRS_HEIGHT + STAIRS_HEIGHT / 2]
            s = [(STAIRS_DEPTH + add), STAIRS_WIDTH, STAIRS_HEIGHT]
            b = Box("s{}".format(i), *p, *s)
            map.add_shape(b)

        return map


class RandomBlocksMap(ExampleMap):
    def __init__(self, n_blocks: int = 10) -> None:
        super().__init__("Random blocks")
        self.n_blocks = n_blocks

    def setup_map(self) -> MapGenerator:
        map = MapGenerator()

        BLOCK_HEIGHT = 0.3
        BLOCK_DEPTH = 1.0
        BLOCK_WIDTH = 1.0

        self._add_floor(map)

        for i in range(self.n_blocks):
            d = random.uniform(0, BLOCK_DEPTH)
            w = random.uniform(0, BLOCK_WIDTH)
            h = random.uniform(0, BLOCK_HEIGHT)

            x = random.uniform(-2, 2)
            y = random.uniform(-2, 2)
            z = h / 2
            p = [x, y, z]
            s = [d, w, h]
            b = Box("b{}".format(i), *p, *s)
            map.add_shape(b)

        return map


AVAILABLE_MAPS = {
    "StairsMap": StairsMap,
    "RandomBlocksMap": RandomBlocksMap,
    "TestMap": TestMap,
}
