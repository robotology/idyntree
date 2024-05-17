# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

from enum import Enum
import os
import idyntree.bindings as idyn
import numpy as np
import warnings
from pathlib import Path


class Line:
    """
    A class representing a line
    """
    def __init__(self, vertices, linewidth=1.0):
        self.vertices = vertices
        self.linewidth = linewidth

    def isBox(self):
        return False

    def isSphere(self):
        return False

    def isCylinder(self):
        return False


class LinkVisualizedObject:
    """
    A class representing storing the data about a link visualized object
    """
    class Type(Enum):
        MESH = 0
        PRIMITIVE_GEOMETRY = 1

    def __init__(self, name, meshcat_object, material, solid_shape, viz_type):
        self.name = name
        self.meshcat_object = meshcat_object
        self.material = material
        self.solid_shape = solid_shape
        self.type = viz_type


class MeshcatVisualizer:
    """
    A simple wrapper to the meshcat visualizer. The MeshcatVisualizer class is highly inspired by the Pinocchio version
    of the MeshCat visualizer
    https://github.com/stack-of-tasks/pinocchio/blob/b134b25f1409f5bf036105b996da2d29c1a66a12/bindings/python/pinocchio/visualize/meshcat_visualizer.py
    """

    def __init__(self, zmq_url=None):
        try:
            import meshcat
        except ModuleNotFoundError:
            raise ModuleNotFoundError("the meshcat module is not found by the idyntree MeshcatVisualizer. Please install explicitly meshcat-python (as it is not an idyntree dependency) and try again.")

        if zmq_url is not None:
            print("Connecting to meshcat-server at zmq_url=" + zmq_url + ".")

        self.viewer = meshcat.Visualizer(zmq_url=zmq_url)
        self.traversal = dict()
        self.model = dict()
        self.link_pos = dict()
        self.primitive_geometries_names = []
        self.arrow_names = []
        self.link_visuals = dict()

    @staticmethod
    def __is_mesh(geometry_object: idyn.SolidShape) -> bool:

        if not geometry_object.isExternalMesh():
            return False
        mesh_path = geometry_object.asExternalMesh().getFileLocationOnLocalFileSystem()

        # Check whether the geometry object contains a Mesh supported by MeshCat
        if mesh_path == "":
            return False

        _, file_extension = os.path.splitext(mesh_path)
        if file_extension.lower() in [".dae", ".obj", ".stl"]:
            return True

        return False

    @staticmethod
    def __skew(x):
        return np.array([[0, -x[2], x[1]],
                         [x[2], 0, -x[0]],
                         [-x[1], x[0], 0]])

    @staticmethod
    def __load_mesh(geometry_object: idyn.SolidShape):
        import meshcat

        mesh_path = geometry_object.asExternalMesh().getFileLocationOnLocalFileSystem()

        # try to import the mesh
        if mesh_path == "":
            return None

        _, file_extension = os.path.splitext(mesh_path)

        basename = os.path.basename(mesh_path)
        file_name = os.path.splitext(basename)[0]

        geometry_object.asExternalMesh().setName(file_name)

        if file_extension.lower() == ".dae":
            obj = meshcat.geometry.DaeMeshGeometry.from_file(mesh_path)
        elif file_extension.lower() == ".obj":
            obj = meshcat.geometry.ObjMeshGeometry.from_file(mesh_path)
        elif file_extension.lower() == ".stl":
            obj = meshcat.geometry.StlMeshGeometry.from_file(mesh_path)
        else:
            msg = "The following mesh cannot be loaded: {}.".format(mesh_path)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    @staticmethod
    def __load_primitive_geometry(geometry_object: idyn.SolidShape):
        import meshcat

        # Cylinders need to be rotated
        R = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        RotatedCylinder = type(
            "RotatedCylinder",
            (meshcat.geometry.Cylinder,),
            {"intrinsic_transform": lambda self: R},
        )
        if geometry_object.isCylinder():
            obj = RotatedCylinder(
                geometry_object.asCylinder().getLength(),
                geometry_object.asCylinder().getRadius(),
            )
        elif geometry_object.isBox():
            obj = meshcat.geometry.Box(
                (
                    geometry_object.asBox().getX(),
                    geometry_object.asBox().getY(),
                    geometry_object.asBox().getZ(),
                )
            )
        elif geometry_object.isSphere():
            obj = meshcat.geometry.Sphere(geometry_object.asSphere().getRadius())
        elif isinstance(geometry_object, Line):
            # transpose the vertices to have the correct shape
            obj = meshcat.geometry.PointsGeometry(geometry_object.vertices)
        else:
            msg = "Unsupported geometry type (%s)" % type(geometry_object)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def __apply_transform_to_primitive_geomety(
        self, world_H_frame, solid_shape, viewer_name
    ):
        world_H_geometry = (
            (world_H_frame * solid_shape.getLink_H_geometry())
            .asHomogeneousTransform()
            .toNumPy()
        )
        world_H_geometry_scaled = np.array(world_H_geometry)

        # Update viewer configuration.
        self.viewer[viewer_name].set_transform(world_H_geometry_scaled)

    def __apply_transform(self, world_H_frame, solid_shape, viewer_name):
        world_H_geometry = (
            (world_H_frame * solid_shape.getLink_H_geometry())
            .asHomogeneousTransform()
            .toNumPy()
        )
        scale = list(solid_shape.asExternalMesh().getScale().toNumPy().flatten())
        extended_scale = np.diag(np.concatenate((scale, [1.0])))
        world_H_geometry_scaled = np.array(world_H_geometry).dot(extended_scale)

        # Update viewer configuration.
        self.viewer[viewer_name].set_transform(world_H_geometry_scaled)

    def __model_exists(self, model_name):
        return (
            model_name in self.model.keys()
            or model_name in self.traversal.keys()
            or model_name in self.link_pos.keys()
        )

    def __primitive_geometry_exists(self, geometry_name: str):
        return geometry_name in self.primitive_geometries_names

    def __arrow_exists(self, arrow_name: str):
        return arrow_name in self.arrow_names

    def __get_color_from_shape(self, solid_shape, color):
        if color is None:
            mesh_color = solid_shape.getMaterial().color()
        elif isinstance(color, float):
            mesh_color = solid_shape.getMaterial().color()
            mesh_color[3] = color
        elif isinstance(color, list):
            if len(color) == 4:
                mesh_color = color
            elif len(color) == 3:
                mesh_color = color
                mesh_color.append(1.0)
            else:
                mesh_color = [1.0, 1.0, 1.0, 1.0]
                msg = "Not compatible color type. Please pass a list if you want to specify the rgb and the alpha or just a float to specify the alpha channel."
                warnings.warn(msg, category=UserWarning, stacklevel=2)
        else:
            mesh_color = [1.0, 1.0, 1.0, 1.0]
            msg = "Not compatible color type. Please pass a list if you want to specify the rgb and the alpha or just a float to specify the alpha channel."
            warnings.warn(msg, category=UserWarning, stacklevel=2)

        return mesh_color

    def __add_model_geometry_to_viewer(
        self, model_geometry: idyn.ModelSolidShapes, model_name: str, color
    ):
        import meshcat

        if not self.__model_exists(model_name):
            msg = "The model named: " + model_name + " does not exist."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        # Solve forward kinematics
        joint_pos = idyn.VectorDynSize(self.model[model_name].getNrOfJoints())
        joint_pos.zero()
        idyn.ForwardPositionKinematics(
            self.model[model_name],
            self.traversal[model_name],
            idyn.Transform.Identity(),
            joint_pos,
            self.link_pos[model_name],
        )

        link_solid_shapes = model_geometry.getLinkSolidShapes()

        self.link_visuals[model_name] = dict()

        for link_index in range(0, self.model[model_name].getNrOfLinks()):

            world_H_frame = self.link_pos[model_name](link_index)
            link_name = self.model[model_name].getLinkName(link_index)
            self.link_visuals[model_name][link_index] = []

            for geom in range(0, len(link_solid_shapes[link_index])):
                solid_shape = model_geometry.getLinkSolidShapes()[link_index][geom]

                if self.__is_mesh(solid_shape):
                    obj = self.__load_mesh(solid_shape)
                    viz_type = LinkVisualizedObject.Type.MESH
                else:
                    obj = self.__load_primitive_geometry(solid_shape)
                    viz_type = LinkVisualizedObject.Type.PRIMITIVE_GEOMETRY

                if obj is None:
                    msg = (
                        "The geometry object named "
                        + solid_shape.asExternalMesh().getName()
                        + " is not valid."
                    )
                    warnings.warn(msg, category=UserWarning, stacklevel=2)
                    continue

                if viz_type is LinkVisualizedObject.Type.MESH:
                    viewer_name = (
                        model_name
                        + "/"
                        + link_name
                        + "/"
                        + solid_shape.asExternalMesh().getName()
                    )
                else:
                    viewer_name = model_name + "/" + link_name + "/geometry" + str(geom)

                material = None
                if isinstance(obj, meshcat.geometry.Object):
                    self.viewer[viewer_name].set_object(obj)
                elif isinstance(obj, meshcat.geometry.Geometry):
                    material = meshcat.geometry.MeshPhongMaterial()

                    mesh_color = self.__get_color_from_shape(solid_shape, color)
                    material.color = (
                        int(mesh_color[0] * 255) * 256 ** 2
                        + int(mesh_color[1] * 255) * 256
                        + int(mesh_color[2] * 255)
                    )

                    # Add transparency, if needed.
                    if float(mesh_color[3]) != 1.0:
                        material.transparent = True
                        material.opacity = float(mesh_color[3])

                    self.viewer[viewer_name].set_object(obj, material)

                self.link_visuals[model_name][link_index].append(
                    LinkVisualizedObject(name=viewer_name,
                                         meshcat_object=obj,
                                         material=material,
                                         solid_shape=solid_shape,
                                         viz_type=viz_type)
                )

                if viz_type is LinkVisualizedObject.Type.MESH:
                    self.__apply_transform(world_H_frame, solid_shape, viewer_name)
                else:
                    self.__apply_transform_to_primitive_geomety(
                        world_H_frame, solid_shape, viewer_name
                    )

    def set_multibody_system_state(
        self, base_position, base_rotation, joint_value, model_name="iDynTree"
    ):
        """Display the robot at given configuration."""

        if not self.__model_exists(model_name):
            msg = "The multi-body system named: " + model_name + " does not exist."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        base_rotation_idyn = idyn.Rotation()
        base_position_idyn = idyn.Position()
        base_pose_idyn = idyn.Transform()

        for i in range(0, 3):
            base_position_idyn.setVal(i, base_position[i])
            for j in range(0, 3):
                base_rotation_idyn.setVal(i, j, base_rotation[i, j])

        base_pose_idyn.setRotation(base_rotation_idyn)
        base_pose_idyn.setPosition(base_position_idyn)

        if len(joint_value) != self.model[model_name].getNrOfJoints():
            msg = "The size of the joint_values is different from the model DoFs"
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        joint_pos_idyn = idyn.VectorDynSize(self.model[model_name].getNrOfJoints())
        for i in range(0, self.model[model_name].getNrOfJoints()):
            joint_pos_idyn.setVal(i, joint_value[i])

        # Solve forward kinematics
        idyn.ForwardPositionKinematics(
            self.model[model_name],
            self.traversal[model_name],
            base_pose_idyn,
            joint_pos_idyn,
            self.link_pos[model_name],
        )

        # Update the visual shapes
        for link_index in range(0, self.model[model_name].getNrOfLinks()):

            if link_index not in self.link_visuals[model_name]:
                continue

            for link_visual in self.link_visuals[model_name][link_index]:
                if link_visual.type is LinkVisualizedObject.Type.MESH:
                    self.__apply_transform(
                        self.link_pos[model_name](link_index),
                        link_visual.solid_shape,
                        link_visual.name,
                    )
                else:
                    self.__apply_transform_to_primitive_geomety(
                        self.link_pos[model_name](link_index),
                        link_visual.solid_shape,
                        link_visual.name,
                    )

    def set_link_color(self, model_name, link_name, color):
        if not isinstance(color, list) and not isinstance(color, float):
            msg = "The color must be a list or a float."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        if isinstance(color, list) and len(color) != 3 and len(color) != 4:
            msg = "The color must be a list of 3 or 4 elements."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        if not self.__model_exists(model_name):
            msg = "The multi-body system named: " + model_name + " does not exist."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        link_index = self.model[model_name].getLinkIndex(link_name)

        if link_index < 0:
            msg = "The link named: " + link_name + " does not exist."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        if link_index not in self.link_visuals[model_name]:
            msg = "The link named: " + link_name + " does not have any visual geometry."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        has_material = False
        for link_visual in self.link_visuals[model_name][link_index]:
            if link_visual.material is None:
                continue

            has_material = True
            if (isinstance(color, float) or
               (isinstance(color, list) and len(color) == 4)):
                opacity = color if isinstance(color, float) else color[3]
                link_visual.material.transparent = float(opacity) != 1.0
                link_visual.material.opacity = float(opacity)
            if isinstance(color, list) and len(color) >= 3:
                link_visual.material.color = (
                    int(color[0] * 255) * 256 ** 2
                    + int(color[1] * 255) * 256
                    + int(color[2] * 255)
                )
            self.viewer[link_visual.name].set_object(link_visual.meshcat_object,
                                                     link_visual.material)

        if not has_material:
            msg = "The link named: " + link_name + " does not have any material."
            warnings.warn(msg, category=UserWarning, stacklevel=2)

    def set_model_color(self, model_name, color):

        if not self.__model_exists(model_name):
            msg = "The multi-body system named: " + model_name + " does not exist."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        for link_index in range(self.model[model_name].getNrOfLinks()):
            self.set_link_color(model_name,
                                self.model[model_name].getLinkName(link_index),
                                color)

    def open(self):
        self.viewer.open()

    def jupyter_cell(self):
        return self.viewer.jupyter_cell()

    def load_primitive_geometry(self, solid_shape, shape_name="iDynTree", color=None):
        import meshcat

        # check if the model already exist
        if self.__primitive_geometry_exists(shape_name) or self.__model_exists(
            shape_name
        ):
            msg = "The model named: " + shape_name + " already exists."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        # try to load the primitive
        obj = self.__load_primitive_geometry(solid_shape)

        if obj is None:
            msg = "The geometry object named " + shape_name + " is not valid."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        viewer_name = shape_name

        if isinstance(obj, meshcat.geometry.Object):
            self.viewer[viewer_name].set_object(obj)
        elif isinstance(obj, meshcat.geometry.Geometry) or isinstance(obj, meshcat.geometry.PointsGeometry):
            if not isinstance(obj, meshcat.geometry.PointsGeometry):
                material = meshcat.geometry.MeshPhongMaterial()
            else:
                material = meshcat.geometry.MeshBasicMaterial()

            mesh_color = self.__get_color_from_shape(solid_shape, color)
            material.color = (
                int(mesh_color[0] * 255) * 256 ** 2
                + int(mesh_color[1] * 255) * 256
                + int(mesh_color[2] * 255)
            )

            # Add transparency, if needed.
            if float(mesh_color[3]) != 1.0:
                material.transparent = True
                material.opacity = float(mesh_color[3])

            # If it is a line we need to manually set the material
            if isinstance(obj, meshcat.geometry.PointsGeometry):
                material.linewidth = solid_shape.linewidth
                self.viewer[viewer_name].set_object(meshcat.geometry.Line(obj, material))
            else:
                self.viewer[viewer_name].set_object(obj, material)

            self.primitive_geometries_names.append(viewer_name)

    def set_primitive_geometry_transform(
        self, position, rotation, shape_name="iDynTree"
    ):
        if self.__primitive_geometry_exists(shape_name):
            transform = np.zeros((4, 4))
            transform[0:3, 0:3] = rotation
            transform[0:3, 3] = position
            transform[3, 3] = 1
            self.viewer[shape_name].set_transform(transform)

    def set_arrow_transform(self, origin, vector, shape_name="iDynTree"):
        if not self.__arrow_exists(shape_name):
            warnings.warn("The arrow named: " + shape_name + " does not exist.", category=UserWarning, stacklevel=2)
            return

        # compute the scaling matrix
        S = np.diag([1, 1, np.linalg.norm(vector)])
        transform = np.zeros((4, 4))
        transform[0:3, 3] = np.array(origin) + np.array(vector) / 2
        transform[3, 3] = 1

        if np.linalg.norm(vector) < 1e-6:
            self.viewer[shape_name].set_transform(transform)
            return

        # extract rotation matrix from a normalized vector
        vector = vector / np.linalg.norm(vector)
        dummy_vector = np.array([0, 0, 1])

        # compute the rotation matrix between the two vectors
        # math taken from
        # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        v = np.cross(dummy_vector, vector)
        s = np.linalg.norm(v)
        if s < 1e-6:
            R = np.eye(3)
        else:
            c = np.dot(dummy_vector, vector)
            skew_symmetric_matrix = self.__skew(v)
            R = np.eye(3) + skew_symmetric_matrix + np.dot(skew_symmetric_matrix, skew_symmetric_matrix) * ((1 - c) / (s ** 2))

        transform[0:3, 0:3] = R @ S
        self.viewer[shape_name].set_transform(transform)

    def load_model_from_file(
        self, model_path: str, considered_joints=None, model_name="iDynTree", color=None
    ):

        p = Path(model_path)
        path_str = str(p.absolute())
        model_loader = idyn.ModelLoader()
        if considered_joints is None:
            ok = model_loader.loadModelFromFile(path_str)

        else:
            considered_joints_idyn = idyn.StringVector()
            for joint in considered_joints:
                considered_joints_idyn.push_back(joint)

            ok = model_loader.loadReducedModelFromFile(path_str, considered_joints_idyn)

        if not ok:
            msg = (
                "Unable to load the model named: "
                + model_name
                + " from the file: "
                + model_path
                + "."
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        self.load_model(model=model_loader.model(), model_name=model_name, color=color)

    def load_model(self, model: idyn.Model, model_name="iDynTree", color=None):

        # check if the model already exist
        if self.__model_exists(model_name):
            msg = "The model named: " + model_name + " already exists."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        self.model[model_name] = model.copy()
        self.traversal[model_name] = idyn.Traversal()
        self.link_pos[model_name] = idyn.LinkPositions()

        self.model[model_name].computeFullTreeTraversal(self.traversal[model_name])
        self.link_pos[model_name].resize(self.model[model_name])

        self.__add_model_geometry_to_viewer(
            model_geometry=self.model[model_name].visualSolidShapes(),
            model_name=model_name,
            color=color,
        )

    def load_sphere(self, radius, shape_name="iDynTree", color=None):
        sphere = idyn.Sphere()
        sphere.setRadius(radius)
        self.load_primitive_geometry(
            solid_shape=sphere, shape_name=shape_name, color=color
        )

    def load_cylinder(self, radius, length, shape_name="iDynTree", color=None):
        cylinder = idyn.Cylinder()
        cylinder.setRadius(radius)
        cylinder.setLength(length)
        self.load_primitive_geometry(
            solid_shape=cylinder, shape_name=shape_name, color=color
        )

    def load_arrow(self, radius, shape_name="iDynTree", color=None):
        self.load_cylinder(radius, 1.0, shape_name=shape_name, color=color)
        if self.__primitive_geometry_exists(shape_name):
            self.arrow_names.append(shape_name)

    def load_box(self, x, y, z, shape_name="iDynTree", color=None):
        box = idyn.Box()
        box.setX(x)
        box.setY(y)
        box.setZ(z)
        self.load_primitive_geometry(
            solid_shape=box, shape_name=shape_name, color=color
        )

    def load_line(self, vertices, linewidth, shape_name="iDynTree", color=None):
        if vertices.shape[0] != 3:
            msg = "The vertices must be a 3xN matrix."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return
        line = Line(vertices=vertices, linewidth=linewidth)
        self.load_primitive_geometry(
            solid_shape=line, shape_name=shape_name, color=color
        )

    def delete(self, shape_name="iDynTree"):
        if self.__primitive_geometry_exists(shape_name):
            self.viewer[shape_name].delete()
            self.primitive_geometries_names.remove(shape_name)
        elif self.__arrow_exists(shape_name):
            self.viewer[shape_name].delete()
            self.arrow_names.remove(shape_name)
        elif self.__model_exists(shape_name):
            self.viewer[shape_name].delete()
            del self.model[shape_name]
            del self.traversal[shape_name]
            del self.link_pos[shape_name]
        else:
            msg = "The object named: " + shape_name + " does not exist."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
