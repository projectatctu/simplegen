import os
import time
import rospy

import lxml.etree as ET
from shapes import Shape, Box, ShapeTypes

from typing import List

from visualization_msgs.msg import Marker, MarkerArray

from abc import ABC, abstractmethod


class MapGenerator:
    PLY_HEADER = """ply
    format ascii 1.0
    element vertex {}
    property float x      
    property float y           
    property float z 
    element face {}  
    property list uchar int vertex_index
    end_header
    """

    def __init__(self) -> None:
        """Initialize map generator with an empty list of shapes"""
        self.shapes: List[Shape] = list()

    def add_shape(self, shape: Shape) -> None:
        """Add a shape to the map

        Args:
            shape (Shape): Shape to be added
        """
        self.shapes.append(shape)

    def generate_world(self, filename: str) -> None:
        """Generate a gazebo world file with the given filename and previously added shapes

        Args:
            filename (str): Name of the world file
        """
        # Create root element
        root = ET.Element("sdf", version="1.6")
        world = ET.SubElement(root, "world", name="generated_world")

        # Add ground plane
        self._add_ground_plane(world)

        # Add the rest of shapes
        for shape in self.shapes:
            self._add_shape(world, shape)

        # Generate xml
        with open(filename, "wb") as f:
            f.write(ET.tostring(root, pretty_print=True))

    def generate_ply(self, filename: str) -> None:
        """Generate a ply file with the given filename and previously added shapes

        Args:
            filename (str): Name of the ply file
        """
        n_vertices = sum([shape.n_vertices for shape in self.shapes])
        n_faces = sum([shape.n_faces for shape in self.shapes])

        header = self.PLY_HEADER.format(n_vertices, n_faces)

        with open(filename, "w") as f:
            f.write(header)

            # Generate vertices
            for shape in self.shapes:
                for vertex in shape.get_vertices():
                    f.write(vertex.ply())

            # Generate faces
            offset = 0
            for shape in self.shapes:
                for face in shape.get_faces():
                    f.write(face.ply(offset))
                offset += shape.n_vertices

    def _add_ground_plane(self, world: ET.SubElement) -> None:
        # xml_set_value(world, "include/uri", "model://ground_plane")
        pass

    def _add_shape(self, world: ET.SubElement, shape: Shape) -> None:
        xml_elem = shape.generate_xml()
        world.append(xml_elem)


class Map(ABC):
    def __init__(self, description: str) -> None:
        """Initialize map with a description

        Args:
            description (str): map description
        """
        self.description = description
        self.map_generator = self.setup_map()

    @abstractmethod
    def setup_map(self) -> MapGenerator:
        """Setup map and prepare map generator

        Returns:
            MapGenerator: prepared map generator
        """
        pass

    def _create_folder(self, filename: str) -> None:
        """Generate folder for file if it doesn't exist

        Args:
            filename (str): file path
        """
        path, _ = os.path.split(filename)
        if not os.path.exists(path):
            os.makedirs(path)

    def save_map(self, filename: str) -> None:
        """Save map into a world file

        Args:
            filename (str): world filename
        """
        self._create_folder(filename)
        self.map_generator.generate_world(filename)

    def save_ply(self, filename: str) -> None:
        """Save map's ply

        Args:
            filename (str): ply filename
        """
        self._create_folder(filename)
        self.map_generator.generate_ply(filename)

    def rviz_visualize(self) -> None:
        """Visualize the map in rviz"""

        # Initialize ros node
        rospy.init_node("simplegen_rviz_visualizer")

        # Create marker publisher
        marker_publisher = rospy.Publisher("markers", MarkerArray, queue_size=1)

        # Delete all markers
        self._delete_all_markers(marker_publisher)
        time.sleep(0.2)

        # Publish new markers
        self._publish_rviz_markers(marker_publisher)
        time.sleep(0.2)

    def _delete_all_markers(self, marker_publisher: rospy.Publisher) -> None:
        """Delete all markers in rviz

        Args:
            marker_publisher (rospy.Publisher): ROS publisher of MarkerArray message
        """
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "map"
        delete_all_marker.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(delete_all_marker)
        marker_publisher.publish(markers)

    def _publish_rviz_markers(self, marker_publisher: rospy.Publisher) -> None:
        """Publish markers to rviz

        Args:
            marker_publisher (rospy.Publisher): ROS publisher of MarkerArray message
        """
        markers = MarkerArray()
        stamp = rospy.Time.now()
        for i, shape in enumerate(self.map_generator.shapes):
            markers.markers.append(shape.get_marker(world_frame="map", idx=i, stamp=stamp))
        marker_publisher.publish(markers)


class MapReader:
    def __init__(self, filename: str) -> None:
        """Read a world file

        Args:
            filename (str): world file path
        """
        self.filename = filename
        root = self._read()
        self._shapes = self._extract_shapes(root)

    def get_shapes(self) -> List[Shape]:
        """Return a list of extracted shapes

        Returns:
            List[Shape]: Shapes found in the world file
        """
        return self._shapes

    def _read(self):
        tree = ET.parse(self.filename)
        root = tree.getroot()
        return root

    def _extract_shapes(self, root: ET.Element) -> List[Shape]:
        shapes = list()
        shapes.extend(self._extract_boxes(root))
        return shapes

    def _extract_boxes(self, root: ET.Element) -> List[Box]:
        boxes = list()
        for model in root.iter("model"):
            shape_type = self._get_shape_type(model)
            if shape_type != ShapeTypes.BOX:
                continue

            name = model.attrib["name"]
            visualize = not name.endswith(Shape.DONT_VISUALIZE)
            if not visualize:
                name = name[: -len(Shape.DONT_VISUALIZE)]
            position = list(map(float, model.find("pose").text.split()[:3]))
            size = list(map(float, model.find("link/collision/geometry/box/size").text.split()))
            box = Box(name, *position, *size, visualize=visualize)
            boxes.append(box)
        return boxes

    def _get_shape_type(self, model: ET.Element) -> ShapeTypes:
        if model.findall("box") is not None:
            return ShapeTypes.BOX


class FromFileMap(Map):
    def __init__(self, filename: str, description: str) -> None:
        """Initialize map from a world file

        Args:
            filename (str): world file path
            description (str): map description
        """
        super().__init__(description)
        self.filename = filename

    def setup_map(self) -> MapGenerator:
        map_reader = MapReader(self.filename)
        shapes = map_reader.get_shapes()
        map = MapGenerator()
        for shape in shapes:
            map.add_shape(shape)
        return map
