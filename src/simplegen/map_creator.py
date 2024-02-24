#!/usr/bin/env python3

import rospy

# Rviz markers
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl

from .map import MapGenerator
from .shapes import Box

import copy

import numpy as np


from scipy.spatial.transform import Rotation as R


class RvizMapCreator:
    def __init__(self):
        self.map = MapGenerator()

        # marker server
        self.server = InteractiveMarkerServer("map_creator")
        self.menuHandler = MenuHandler()
        self.initializeMenuHandler()

        self.interactiveMarkers = list()
        self.markers = list()
        self.shapes = list()
        self.resizeFlags = list()
        self.staticFlags = list()
        self.visualizeFlags = list()

        # Resize parameters
        self.fixed_x_pos = list()
        self.fixed_y_pos = list()
        self.fixed_z_pos = list()
        self.fixed_x_ori = list()
        self.fixed_y_ori = list()
        self.fixed_z_ori = list()
        self.fixed_w_ori = list()

        # TODO: Remove, only for debugging
        for i in range(1):
            self.addBox()
            self.server.insert(self.interactiveMarkers[i], self.processFeedback)
            self.menuHandler.apply(self.server, self.interactiveMarkers[i].name)
        self.server.applyChanges()

    def addBox(self):
        # Generate box shape
        boxId = len(self.shapes)
        box = Box(f"box{boxId}", *[0, 0, 0], *[0.8, 0.2, 0.2], visualize=True)
        self.shapes.append(box)

        # Generate box marker
        marker = box.get_marker(world_frame="map", id=boxId, stamp=rospy.Time.now())
        self.markers.append(marker)

        # Generate box interactive marker
        interactiveMarker = self.getInteractiveMarker(box, marker)
        self.interactiveMarkers.append(interactiveMarker)

        # Set flags
        self.resizeFlags.append(False)
        self.staticFlags.append(True)
        self.visualizeFlags.append(True)

        # Update fixed position and orientation
        self.fixed_x_pos.append(box.x_pos)
        self.fixed_y_pos.append(box.y_pos)
        self.fixed_z_pos.append(box.z_pos)

        quat = box._quaternion
        self.fixed_x_ori.append(quat[0])
        self.fixed_y_ori.append(quat[1])
        self.fixed_z_ori.append(quat[2])
        self.fixed_w_ori.append(quat[3])

        # Sanity check
        assert len(self.interactiveMarkers) == len(self.markers)
        assert len(self.interactiveMarkers) == len(self.shapes)
        assert len(self.interactiveMarkers) == len(self.resizeFlags)
        assert len(self.interactiveMarkers) == len(self.staticFlags)
        assert len(self.interactiveMarkers) == len(self.visualizeFlags)
        assert len(self.interactiveMarkers) == len(self.fixed_x_pos)
        assert len(self.interactiveMarkers) == len(self.fixed_y_pos)
        assert len(self.interactiveMarkers) == len(self.fixed_z_pos)
        assert len(self.interactiveMarkers) == len(self.fixed_x_ori)
        assert len(self.interactiveMarkers) == len(self.fixed_y_ori)
        assert len(self.interactiveMarkers) == len(self.fixed_z_ori)
        assert len(self.interactiveMarkers) == len(self.fixed_w_ori)

    def getInteractiveMarker(self, shape, marker):
        interactiveMarker = InteractiveMarker()
        interactiveMarker.header.frame_id = "map"
        interactiveMarker.name = shape.name
        interactiveMarker.description = shape.name

        # Add simple controls
        buttonControl = InteractiveMarkerControl()
        buttonControl.interaction_mode = InteractiveMarkerControl.BUTTON
        buttonControl.always_visible = True
        buttonControl.markers.append(marker)

        interactiveMarker.controls.append(buttonControl)

        buttonControl = InteractiveMarkerControl()
        buttonControl.interaction_mode = InteractiveMarkerControl.BUTTON
        buttonControl.always_visible = False
        buttonControl.markers.append(marker)

        interactiveMarker.controls.append(buttonControl)

        return interactiveMarker

    def hasMoveControls(self, interactiveMarker):
        for control in interactiveMarker.controls:
            if control.name in ["move_x", "move_y", "move_z"]:
                return True
        return False

    def hasResizeControls(self, interactiveMarker):
        return self.hasMoveControls(interactiveMarker)

    def addResizeControls(self, interactiveMarker):
        assert not self.hasResizeControls(interactiveMarker)
        name = interactiveMarker.name

        resizeXControl = self.getResizeControl(x=True)
        resizeYControl = self.getResizeControl(y=True)
        resizeZControl = self.getResizeControl(z=True)

        interactiveMarker.controls.append(resizeXControl)
        interactiveMarker.controls.append(resizeYControl)
        interactiveMarker.controls.append(resizeZControl)

        self.server.erase(name)
        self.server.applyChanges()
        self.server.insert(interactiveMarker, self.processFeedback)
        self.server.applyChanges()

    def addMoveControls(self, interactiveMarker):
        assert not self.hasMoveControls(interactiveMarker)
        name = interactiveMarker.name

        moveXControl = self.getTranslationControl(x=True, fixed=True)
        moveYControl = self.getTranslationControl(y=True, fixed=True)
        moveZControl = self.getTranslationControl(z=True, fixed=True)

        rotateXControl = self.getRotationControl(x=True, fixed=True)
        rotateYControl = self.getRotationControl(y=True, fixed=True)
        rotateZControl = self.getRotationControl(z=True, fixed=True)

        interactiveMarker.controls.append(moveXControl)
        interactiveMarker.controls.append(moveYControl)
        interactiveMarker.controls.append(moveZControl)

        interactiveMarker.controls.append(rotateXControl)
        interactiveMarker.controls.append(rotateYControl)
        interactiveMarker.controls.append(rotateZControl)

        self.server.erase(name)
        self.server.applyChanges()
        self.server.insert(interactiveMarker, self.processFeedback)
        self.server.applyChanges()

    def removeMoveControls(self, interactiveMarker):
        controls = list()
        for c in interactiveMarker.controls:
            if c.name not in ["move_x", "move_y", "move_z", "rotate_x", "rotate_y", "rotate_z"]:
                controls.append(c)
        interactiveMarker.controls = controls

        self.server.erase(interactiveMarker.name)
        self.server.applyChanges()
        self.server.insert(interactiveMarker, self.processFeedback)
        self.server.applyChanges()

    def removeResizeControls(self, interactiveMarker):
        controls = list()
        for c in interactiveMarker.controls:
            if c.name not in ["move_x", "move_y", "move_z"]:
                controls.append(c)
        interactiveMarker.controls = controls

        self.server.erase(interactiveMarker.name)
        self.server.applyChanges()
        self.server.insert(interactiveMarker, self.processFeedback)
        self.server.applyChanges()

    def resizeCallback(self, feedback):
        markerName = feedback.marker_name
        marker = self.server.get(markerName)
        index = self.findIndex(marker)
        self.resizeFlags[index] = not self.resizeFlags[index]

        if self.resizeFlags[index] is True:
            add = self.hasMoveControls(marker) or self.hasResizeControls(marker)
            self.removeMoveControls(marker)
            self.removeResizeControls(marker)
            if add:
                self.addResizeControls(marker)

        if self.resizeFlags[index] is False:
            add = self.hasMoveControls(marker) or self.hasResizeControls(marker)
            self.removeMoveControls(marker)
            self.removeResizeControls(marker)

            if add:
                self.addMoveControls(marker)

    def staticCallback(self, feedback):
        markerName = feedback.marker_name
        marker = self.server.get(markerName)
        index = self.findIndex(marker)
        self.staticFlags[index] = not self.staticFlags[index]
        self.updateMarkerColor(marker)
        self.server.applyChanges()

    def toggleCallback(self, feedback):
        markerName = feedback.marker_name
        marker = self.server.get(markerName)

        index = self.findIndex(marker)

        if self.hasMoveControls(marker) or self.hasResizeControls(marker):
            self.removeMoveControls(marker)
            self.removeResizeControls(marker)
        else:
            self.removeAll()
            if self.resizeFlags[index]:
                self.addResizeControls(marker)
            else:
                self.addMoveControls(marker)

    def initializeMenuHandler(self):
        resizeButton = self.menuHandler.insert("resize", callback=self.resizeCallback)
        staticButton = self.menuHandler.insert("static", callback=self.staticCallback)
        duplicateButton = self.menuHandler.insert("duplicate", callback=self.duplicateCallback)

        self.menuHandler.setCheckState(resizeButton, MenuHandler.CHECKED)
        self.menuHandler.setCheckState(staticButton, MenuHandler.CHECKED)
        self.menuHandler.setCheckState(duplicateButton, MenuHandler.CHECKED)

    def duplicateCallback(self, feedback):
        markerName = feedback.marker_name
        marker = self.server.get(markerName)
        index = self.findIndex(marker)
        m = self.markers[index]

        self.addBox()
        duplicateMarker = self.interactiveMarkers[-1]

        self.server.insert(duplicateMarker, self.processFeedback)
        self.menuHandler.apply(self.server, duplicateMarker.name)
        self.server.applyChanges()

        duplicateMarker.pose.position.x = marker.pose.position.x
        duplicateMarker.pose.position.y = marker.pose.position.y
        duplicateMarker.pose.position.z = marker.pose.position.z

        duplicateMarker.pose.orientation.x = marker.pose.orientation.x
        duplicateMarker.pose.orientation.y = marker.pose.orientation.y
        duplicateMarker.pose.orientation.z = marker.pose.orientation.z
        duplicateMarker.pose.orientation.w = marker.pose.orientation.w

        dupm = self.markers[-1]
        dupm.scale.x = m.scale.x
        dupm.scale.y = m.scale.y
        dupm.scale.z = m.scale.z

        dupm.pose.position.x = marker.pose.position.x
        dupm.pose.position.y = marker.pose.position.y
        dupm.pose.position.z = marker.pose.position.z

        dupm.pose.orientation.x = marker.pose.orientation.x
        dupm.pose.orientation.y = marker.pose.orientation.y
        dupm.pose.orientation.z = marker.pose.orientation.z
        dupm.pose.orientation.w = marker.pose.orientation.w

        self.fixed_x_pos[-1] = marker.pose.position.x
        self.fixed_y_pos[-1] = marker.pose.position.y
        self.fixed_z_pos[-1] = marker.pose.position.z

        self.fixed_x_ori[-1] = marker.pose.orientation.x
        self.fixed_y_ori[-1] = marker.pose.orientation.y
        self.fixed_z_ori[-1] = marker.pose.orientation.z
        self.fixed_w_ori[-1] = marker.pose.orientation.w

        self.removeAll()
        self.addMoveControls(duplicateMarker)

    def getTranslationControl(self, x=False, y=False, z=False, fixed=False):
        assert x + y + z == 1
        name = "move_" + ("x" if x else ("y" if y else "z"))
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = 1
        control.orientation.x = bool(x)
        control.orientation.y = bool(y)
        control.orientation.z = bool(z)
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED if fixed else InteractiveMarkerControl.INHERIT
        control.always_visible = True
        return control

    def getRotationControl(self, x=False, y=False, z=False, fixed=False):
        assert x + y + z == 1
        name = "rotate_" + ("x" if x else ("y" if y else "z"))
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = 1
        control.orientation.x = bool(x)
        control.orientation.y = bool(y)
        control.orientation.z = bool(z)
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED if fixed else InteractiveMarkerControl.INHERIT
        control.always_visible = True
        return control

    def getResizeControl(self, x=False, y=False, z=False):
        return self.getTranslationControl(x, y, z, fixed=False)

    def findIndex(self, interactiveMarker):
        for i, marker in enumerate(self.interactiveMarkers):
            if marker.name == interactiveMarker.name:
                return i
        assert False

    def processMoveFeedback(self, feedback):
        markerName = feedback.marker_name
        marker = self.server.get(markerName)
        index = self.findIndex(marker)
        m = self.markers[index]

        if self.resizeFlags[index]:
            marker.pose.position.x = self.fixed_x_pos[index]
            marker.pose.position.y = self.fixed_y_pos[index]
            marker.pose.position.z = self.fixed_z_pos[index]
            marker.pose.orientation.x = self.fixed_x_ori[index]
            marker.pose.orientation.y = self.fixed_y_ori[index]
            marker.pose.orientation.z = self.fixed_z_ori[index]
            marker.pose.orientation.w = self.fixed_w_ori[index]
            m.pose.position.x = self.fixed_x_pos[index]
            m.pose.position.y = self.fixed_y_pos[index]
            m.pose.position.z = self.fixed_z_pos[index]
            m.pose.orientation.x = self.fixed_x_ori[index]
            m.pose.orientation.y = self.fixed_y_ori[index]
            m.pose.orientation.z = self.fixed_z_ori[index]
            m.pose.orientation.w = self.fixed_w_ori[index]

            x_diff = feedback.pose.position.x - self.fixed_x_pos[index]
            y_diff = feedback.pose.position.y - self.fixed_y_pos[index]
            z_diff = feedback.pose.position.z - self.fixed_z_pos[index]

            diff_vec = np.array([x_diff, y_diff, z_diff])

            x, y, z, w = (
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w,
            )
            r = R.from_quat([x, y, z, w])
            r = r.as_matrix()
            r = r.T

            diff_vec = np.dot(r, diff_vec)

            m = self.markers[index]
            m.scale.x += diff_vec[0]
            m.scale.y += diff_vec[1]
            m.scale.z += diff_vec[2]

            self.server.insert(marker, self.processFeedback)
        else:
            m.pose.position.x = feedback.pose.position.x
            m.pose.position.y = feedback.pose.position.y
            m.pose.position.z = feedback.pose.position.z
            m.pose.orientation.x = feedback.pose.orientation.x
            m.pose.orientation.y = feedback.pose.orientation.y
            m.pose.orientation.z = feedback.pose.orientation.z
            m.pose.orientation.w = feedback.pose.orientation.w
            self.fixed_x_pos[index] = feedback.pose.position.x
            self.fixed_y_pos[index] = feedback.pose.position.y
            self.fixed_z_pos[index] = feedback.pose.position.z
            self.fixed_x_ori[index] = feedback.pose.orientation.x
            self.fixed_y_ori[index] = feedback.pose.orientation.y
            self.fixed_z_ori[index] = feedback.pose.orientation.z
            self.fixed_w_ori[index] = feedback.pose.orientation.w

            self.server.insert(marker, self.processFeedback)
            print("Marker at position", feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z)

    def processFeedback(self, feedback):
        if feedback.event_type == 3:
            self.toggleCallback(feedback)
        else:
            self.processMoveFeedback(feedback)

        self.menuHandler.reApply(self.server)
        self.server.applyChanges()

    def removeAll(self):
        for marker in self.interactiveMarkers:
            self.removeMoveControls(marker)
            self.removeResizeControls(marker)
        self.server.applyChanges()

    def updateMarkerColor(self, interactiveMarker):
        idx = self.findIndex(interactiveMarker)
        marker = self.markers[idx]
        isStatic = self.staticFlags[idx]

        if isStatic:
            marker.color.r = Box.STATIC_COLOR[0] / 255
            marker.color.g = Box.STATIC_COLOR[1] / 255
            marker.color.b = Box.STATIC_COLOR[2] / 255
        else:
            marker.color.r = Box.NONSTATIC_COLOR[0] / 255
            marker.color.g = Box.NONSTATIC_COLOR[1] / 255
            marker.color.b = Box.NONSTATIC_COLOR[2] / 255

    def updateMap(self):
        for i in range(len(self.shapes)):
            name = self.interactiveMarkers[i].name
            static = self.staticFlags[i]
            visualize = self.visualizeFlags[i]

            marker = self.markers[i]

            self.map.shapes.append(Box.from_marker(marker, name, static, visualize))

    def getMapGenerator(self):
        self.updateMap()
        return self.map
