from map import Map, MapGenerator
from shapes import Box


class ExampleMap(Map):
    def __init__(self, description: str) -> None:
        """Initialize example map with a description

        Args:
            description (str): map description
        """
        super().__init__(description)

    def _add_floor(self, map: MapGenerator) -> None:
        """Add a floor to the map

        Args:
            map (MapGenerator): map generator
        """
        floor = Box("floor", *[0, 0, -0.1], *[5, 5, 0.2], visualize=False)
        map.add_shape(floor)


class StairsMap(ExampleMap):
    def __init__(self, n_stairs: int = 10) -> None:
        super().__init__("Stairs")
        self.n_stairs = n_stairs

    def get_map(self) -> MapGenerator:
        map = MapGenerator()

        # add floor
        self._add_floor(map)

        # TODO(lnotspotl): Make stair parameters configurable
        STAIRS_WIDTH = 3.0
        STAIRS_HEIGHT = 0.2
        STAIRS_DEPTH = 0.3

        # add stairs
        for i in range(self.n_stairs):
            add = 0.0
            if i == (self.n_stairs - 1):
                add = 1.0
            p = [i * STAIRS_DEPTH + add / 2, 0, i * STAIRS_HEIGHT + STAIRS_HEIGHT / 2]
            s = [(STAIRS_DEPTH + add), STAIRS_WIDTH, STAIRS_HEIGHT]
            b = Box("s{}".format(i), *p, *s)
            map.add_shape(b)

        return map
