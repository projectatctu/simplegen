import numpy as np
from enum import Enum
from abc import ABC, abstractmethod
from lxml.etree import Element, SubElement

from scipy.spatial.transform import Rotation

import rospy
from visualization_msgs.msg import Marker

from .utils import xml_set_value

from typing import List


class Vertex:
    def __init__(self, x: float, y: float, z: float) -> None:
        """Initialize a vertex with its position

        Args:
            x (float): x coordinate
            y (float): y coordinate
            z (float): z coordinate
        """
        self._pos = np.array([x, y, z])

    @property
    def position(self) -> np.ndarray:
        """Return vertex position in 3D space

        Returns:
            np.ndarray: vertex position
        """
        return self._pos

    def ply(self) -> str:
        """Return a ply representation of the vertex,

        Returns:
            str: ply representation of the vertex
        """
        x, y, z = self._pos
        return "{} {} {}\n".format(x, y, z)

    def __repr__(self) -> str:
        return f"Vertex({self._pos[0]:.2f}, {self._pos[1]:.2f}, {self._pos[2]:.2f})"


class Face:
    def __init__(self, vertex_idxs: List[int]) -> None:
        self._vertex_idxs = vertex_idxs

    @property
    def idxs(self) -> List[int]:
        """Return vertex indices this face is composed of

        Returns:
            List[int]: vertex indices
        """
        return self._vertex_idxs

    def ply(self, offset: int) -> str:
        """Return a ply representation of the face,

        Args:
            offset (int): Vertex offset

        Returns:
            str: ply representation of the face
        """
        idxs = self.idxs
        return ("{} " * (len(idxs) + 1)).format(len(idxs), *[idx + offset for idx in idxs]) + "\n"

    def __repr__(self) -> str:
        return "Face(idxs={})".format(self._vertex_idxs)


class ShapeTypes(Enum):
    BOX = 1
    CYLINDER = 2
    BALL = 3


class Shape(ABC):
    DONT_VISUALIZE = "DONT_VISUALIZE"
    STATIC_COLOR = (235, 126, 59)  # rgb, orange
    NONSTATIC_COLOR = (29, 113, 181)  # rgb, blue

    def __init__(
        self,
        name: str,
        shape_name: ShapeTypes,
        x_pos: float,
        y_pos: float,
        z_pos: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        static: bool = True,
        visualize: bool = True,
    ) -> None:
        """Initialize shape with its name

        Args:
            name (str): Name
            shape_name (ShapeTypes): Shape type
            x_pos (float): x coordinate of the center of the shape
            y_pos (float): y coordinate of the center of the shape
            z_pos (float): z coordinate of the center of the shape
            roll (float, optional): roll angle. Defaults to 0.0.
            pitch (float, optional): pitch angle. Defaults to 0.0.
            yaw (float, optional): yaw angle. Defaults to 0.0.
            static (bool, optional): Whether the shape is static. Defaults to True.
            visualize (bool, optional): Whether to visualize the shape. Defaults to True.
        """
        self._name = name
        self._shape_name = shape_name
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.z_pos = z_pos
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.static = static
        self.visualize = visualize

    @property
    def name(self) -> str:
        """Return the name of the shape

        Returns:
            str: Name of the shape
        """
        return self._name

    @property
    def type(self) -> ShapeTypes:
        """Return the type of the shape

        Returns:
            ShapeTypes: shape type
        """
        return self._shape_name

    @property
    @abstractmethod
    def marker_type(self) -> int:
        """Return the type of the marker

        Returns:
            int: marker type
        """
        pass

    @abstractmethod
    def generate_xml(self) -> Element:
        """Generate a xml element for the shape, to be used in a world file

        Returns:
            Element: xml representation of the shape
        """
        pass

    @abstractmethod
    def get_vertices(self) -> List[Vertex]:
        """Return the vertices of the shape

        Returns:
            List[Vertex]: list of vertices
        """
        pass

    @abstractmethod
    def get_faces(self) -> List[Face]:
        """Return the faces of the shape

        Returns:
            List[Face]: list of faces
        """
        pass

    @property
    @abstractmethod
    def n_vertices(self) -> int:
        """Return the number of vertices for the given shape

        Returns:
            int: number of shape vertices
        """
        pass

    @property
    @abstractmethod
    def n_faces(self) -> int:
        """Return the number of faces for the given shape

        Returns:
            int: number of shape faces
        """
        pass

    @abstractmethod
    def get_marker(self, world_frame: str, id: int, stamp: rospy.Time) -> Marker:
        """Return a populated marker message for the shapes

        Args:
            world_frame (str): World frame name
            id (int): Marker id
            stamp (rospy.Time): Current ros time

        Returns:
            Marker: Marker message
        """
        pass

    @staticmethod
    @abstractmethod
    def from_marker(marker: Marker, name: str, static: bool = True, visualize: bool = True) -> "Shape":
        """Create a shape from a marker message

        Args:
            marker (Marker): Marker message
            name (str): Shape name
            static (bool, optional): Whether the shape is static. Defaults to True.
            visualize (bool, optional): Whether to visualize the shape. Defaults to True.

        Returns:
            Shape: Shape instance
        """
        pass

    @property
    def _rotation(self) -> Rotation:
        """Return a scipy representation of the box's orientation

        Returns:
            Rotation: box rotation
        """
        return Rotation.from_euler("ZYX", [self.yaw, self.pitch, self.roll], degrees=False)  # ZYX - intrinsic rotation

    @property
    def _rotation_matrix(self) -> np.ndarray:
        """Return the rotation matrix for the box

        Returns:
            np.ndarray: rotation matrix with shape (3, 3)
        """
        return self._rotation.as_matrix()

    @property
    def _quaternion(self) -> np.ndarray:
        """Return the quaternion for the box

        Returns:
            np.ndarray: quaternion with shape (4,) - (x,y,z,w)
        """
        return self._rotation.as_quat()


class Box(Shape):
    TYPE = ShapeTypes.BOX
    N_VERTICES = 8
    N_FACES = 2 * 6  # 2 faces per side - triangle mesh

    def __init__(
        self,
        name: str,
        x_pos: float,
        y_pos: float,
        z_pos: float,
        x_size: float,
        y_size: float,
        z_size: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        static: bool = True,
        visualize: bool = True,
    ) -> None:
        """Initialize a box with its position and size

        Args:
            name (str): Box name
            x_pos (float): x coordinate of the center of the box
            y_pos (float): y coordinate of the center of the box
            z_pos (float): z coordinate of the center of the box
            x_size (float): box size in x direction
            y_size (float): box size in y directionresizeCallback
            z_size (float): box size in z direction
            roll (float, optional): roll angle. Defaults to 0.0.
            pitch (float, optional): pitch angle. Defaults to 0.0.
            yaw (float, optional): yaw angle. Defaults to 0.0.
            static (bool, optional): Whether the box is static. Defaults to True.
            visualize (bool, optional): Whether to visualize the box. Defaults to True.
        """
        super().__init__(
            name, self.__class__.TYPE, x_pos, y_pos, z_pos, roll, pitch, yaw, static=static, visualize=visualize
        )
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size

    @property
    def marker_type(self) -> int:
        return Marker.CUBE

    def generate_xml(self) -> Element:
        model = Element("model", name=self.name + ("" if self.visualize else self.DONT_VISUALIZE))
        xml_set_value(model, "static", "true" if self.static else "false")
        xml_set_value(
            model,
            "pose",
            "{} {} {} {} {} {}".format(self.x_pos, self.y_pos, self.z_pos, self.roll, self.pitch, self.yaw),
        )
        link = SubElement(model, "link", name="link")
        size = "{} {} {}".format(self.x_size, self.y_size, self.z_size)
        xml_set_value(
            link,
            "collision/geometry/box/size",
            size,
            values={"collision": "collision"},
        )
        xml_set_value(link, "visual/geometry/box/size", size, values={"visual": "visual"})
        return model

    def get_vertices(self) -> List[Vertex]:
        """Return the vertices of the box

        Returns:
            List[Vertex]: list of vertices
        """

        def _get_vertex(xmult: float, ymult: float, zmult: float) -> Vertex:
            position_local = np.array(
                [
                    xmult * self.x_size / 2,
                    ymult * self.y_size / 2,
                    zmult * self.z_size / 2,
                ]
            )
            position_global = self._rotation_matrix @ position_local + np.array([self.x_pos, self.y_pos, self.z_pos])
            return Vertex(*position_global)

        return [
            _get_vertex(-1.0, -1.0, -1.0),
            _get_vertex(-1.0, -1.0, 1.0),
            _get_vertex(-1.0, 1.0, 1.0),
            _get_vertex(-1.0, 1.0, -1.0),
            _get_vertex(1.0, -1.0, -1.0),
            _get_vertex(1.0, -1.0, 1.0),
            _get_vertex(1.0, 1.0, 1.0),
            _get_vertex(1.0, 1.0, -1.0),
        ]

    def get_faces(self) -> List[Face]:
        """Return faces of the box defining its mesh

        Returns:
            List[Face]: List of faces
        """
        return [
            Face([0, 1, 2]),
            Face([0, 2, 3]),
            Face([7, 6, 5]),
            Face([7, 5, 4]),
            Face([0, 4, 5]),
            Face([0, 5, 1]),
            Face([1, 5, 6]),
            Face([1, 6, 2]),
            Face([2, 6, 7]),
            Face([2, 7, 3]),
            Face([3, 7, 4]),
            Face([3, 4, 0]),
        ]

    @property
    def n_vertices(self) -> int:
        return self.N_VERTICES

    @property
    def n_faces(self) -> int:
        return self.N_FACES

    def get_marker(self, world_frame: str, id: int, stamp: rospy.Time) -> Marker:
        marker = Marker()
        marker.header.frame_id = world_frame
        marker.header.stamp = stamp
        marker.id = id
        marker.type = self.marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = self.x_pos
        marker.pose.position.y = self.y_pos
        marker.pose.position.z = self.z_pos
        quat = self._quaternion
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = self.x_size
        marker.scale.y = self.y_size
        marker.scale.z = self.z_size
        marker.color.a = 1.0
        marker.color.r = self.STATIC_COLOR[0] / 255 if self.static else self.NONSTATIC_COLOR[0] / 255
        marker.color.g = self.STATIC_COLOR[1] / 255 if self.static else self.NONSTATIC_COLOR[1] / 255
        marker.color.b = self.STATIC_COLOR[2] / 255 if self.static else self.NONSTATIC_COLOR[2] / 255
        return marker

    @staticmethod
    def from_marker(marker: Marker, name: str, static: bool = True, visualize: bool = True) -> "Box":
        x_pos = marker.pose.position.x
        y_pos = marker.pose.position.y
        z_pos = marker.pose.position.z

        yaw, pitch, roll = Rotation.from_quat(
            [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]
        ).as_euler("ZYX")

        x_size = marker.scale.x
        y_size = marker.scale.y
        z_size = marker.scale.z

        return Box(
            name,
            x_pos,
            y_pos,
            z_pos,
            x_size,
            y_size,
            z_size,
            roll,
            pitch,
            yaw,
            static=static,
            visualize=visualize,
        )


class Cylinder(Shape):
    TYPE = ShapeTypes.CYLINDER
    N_VERTICES = 0  # TODO(lnotspotl): Implement mesh generation
    N_FACES = 0

    def __init__(
        self,
        name: str,
        x_pos: float,
        y_pos: float,
        z_pos: float,
        diameter: float,
        length: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        static: bool = True,
        visualize: bool = True,
    ):
        """Initialize a cylinder with its position and size

        Args:
            name (str): Cylinder name
            x_pos (float): x coordinate of the center of the cylinder
            y_pos (float): y coordinate of the center of the cylinder
            z_pos (float): z coordinate of the center of the cylinder
            diameter (float): cylinder's diameter
            length (float): cylinder's length
            roll (float, optional): roll angle. Defaults to 0.0.
            pitch (float, optional): pitch angle. Defaults to 0.0.
            yaw (float, optional): yaw angle. Defaults to 0.0.
            static (bool, optional): _description_. Defaults to True.
            visualize (bool, optional): _description_. Defaults to True.
        """
        super().__init__(
            name, self.__class__.TYPE, x_pos, y_pos, z_pos, roll, pitch, yaw, static=static, visualize=visualize
        )
        self.diameter = diameter
        self.length = length

    @property
    def marker_type(self) -> int:
        return Marker.CYLINDER

    def generate_xml(self) -> Element:
        model = Element("model", name=self.name + ("" if self.visualize else self.DONT_VISUALIZE))
        parameters = {"radius": str(self.diameter / 2), "length": str(self.length)}
        xml_set_value(model, "static", "true" if self.static else "false")
        xml_set_value(
            model,
            "pose",
            "{} {} {} {} {} {}".format(self.x_pos, self.y_pos, self.z_pos, self.roll, self.pitch, self.yaw),
        )
        link = SubElement(model, "link", name="link")
        xml_set_value(
            link,
            "collision/geometry/cylinder",
            parameters,
            values={"collision": "collision"},
        )
        xml_set_value(link, "visual/geometry/cylinder", parameters, values={"visual": "visual"})
        return model

    def get_vertices(self) -> List[Vertex]:
        """Return the vertices of the cylinder

        Returns:
            List[Vertex]: list of vertices
        """
        raise NotImplementedError("Cylinder mesh generation not implemented")

    def get_faces(self) -> List[Face]:
        """Return faces of the cylinder defining its mesh

        Returns:
            List[Face]: List of faces
        """
        raise NotImplementedError("Cylinder mesh generation not implemented")

    @property
    def n_vertices(self) -> int:
        return self.N_VERTICES

    @property
    def n_faces(self) -> int:
        return self.N_FACES

    def get_marker(self, world_frame: str, id: int, stamp: rospy.Time) -> Marker:
        marker = Marker()
        marker.header.frame_id = world_frame
        marker.header.stamp = stamp
        marker.id = id
        marker.type = self.marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = self.x_pos
        marker.pose.position.y = self.y_pos
        marker.pose.position.z = self.z_pos
        quat = self._quaternion
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = self.diameter
        marker.scale.y = self.diameter
        marker.scale.z = self.length
        marker.color.a = 1.0
        marker.color.r = self.STATIC_COLOR[0] / 255 if self.static else self.NONSTATIC_COLOR[0] / 255
        marker.color.g = self.STATIC_COLOR[1] / 255 if self.static else self.NONSTATIC_COLOR[1] / 255
        marker.color.b = self.STATIC_COLOR[2] / 255 if self.static else self.NONSTATIC_COLOR[2] / 255
        return marker
    
    @staticmethod
    def from_marker(marker: Marker, name: str, static: bool = True, visualize: bool = True) -> "Cylinder":
        raise NotImplementedError("Cylinder from marker not implemented")


class Ball(Shape):
    TYPE = ShapeTypes.BALL
    N_VERTICES = 0  # TODO(lnotspotl): Implement mesh generation
    N_FACES = 0

    def __init__(
        self,
        name: str,
        x_pos: float,
        y_pos: float,
        z_pos: float,
        radius: float,
        static: bool = True,
        visualize: bool = True,
    ):
        """Initialize a ball with its position and size

        Args:
            name (str): Ball name
            x_pos (float): x center coordinate of the center of the ball
            y_pos (float): y center coordinate of the center of the ball
            z_pos (float): z center coordinate of the center of the ball
            raidus (float): ball's radius
            static (bool, optional): _description_. Defaults to True.
            visualize (bool, optional): _description_. Defaults to True.
        """
        roll, pitch, yaw = [0] * 3  # for consistency with other shapes
        super().__init__(
            name, self.__class__.TYPE, x_pos, y_pos, z_pos, roll, pitch, yaw, static=static, visualize=visualize
        )
        self.radius = radius

    @property
    def marker_type(self) -> int:
        return Marker.SPHERE

    def generate_xml(self) -> Element:
        model = Element("model", name=self.name + ("" if self.visualize else self.DONT_VISUALIZE))
        xml_set_value(model, "static", "true" if self.static else "false")
        xml_set_value(
            model,
            "pose",
            "{} {} {} {} {} {}".format(self.x_pos, self.y_pos, self.z_pos, self.roll, self.pitch, self.yaw),
        )
        link = SubElement(model, "link", name="link")
        xml_set_value(
            link,
            "collision/geometry/sphere/radius",
            str(self.radius),
            values={"collision": "collision"},
        )
        xml_set_value(link, "visual/geometry/sphere/radius", str(self.radius), values={"visual": "visual"})
        return model

    def get_vertices(self) -> List[Vertex]:
        """Return the vertices of the ball

        Returns:
            List[Vertex]: list of vertices
        """
        raise NotImplementedError("Ball mesh generation not implemented")

    def get_faces(self) -> List[Face]:
        """Return faces of the ball defining its mesh

        Returns:
            List[Face]: List of faces
        """
        raise NotImplementedError("Ball mesh generation not implemented")

    @property
    def n_vertices(self) -> int:
        return self.N_VERTICES

    @property
    def n_faces(self) -> int:
        return self.N_FACES

    def get_marker(self, world_frame: str, id: int, stamp: rospy.Time) -> Marker:
        marker = Marker()
        marker.header.frame_id = world_frame
        marker.header.stamp = stamp
        marker.id = id
        marker.type = self.marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = self.x_pos
        marker.pose.position.y = self.y_pos
        marker.pose.position.z = self.z_pos
        marker.scale.x = self.radius * 2  # diameter
        marker.scale.y = self.radius * 2
        marker.scale.z = self.radius * 2
        marker.color.a = 1.0
        marker.color.r = self.STATIC_COLOR[0] / 255 if self.static else self.NONSTATIC_COLOR[0] / 255
        marker.color.g = self.STATIC_COLOR[1] / 255 if self.static else self.NONSTATIC_COLOR[1] / 255
        marker.color.b = self.STATIC_COLOR[2] / 255 if self.static else self.NONSTATIC_COLOR[2] / 255
        return marker

    @staticmethod
    def from_marker(marker: Marker, name: str, static: bool = True, visualize: bool = True) -> "Sphere":
        raise NotImplementedError("Ball from marker not implemented")