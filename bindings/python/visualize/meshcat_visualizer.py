"""
Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia

Licensed under either the GNU Lesser General Public License v3.0 :
https://www.gnu.org/licenses/lgpl-3.0.html
or the GNU Lesser General Public License v2.1 :
https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
at your option.
"""

import os
import idyntree.bindings as idyn
import numpy as np
import warnings


class MeshcatVisualizer:
    """
    A simple wrapper to the meshcat visualizer. The MeshcatVisualizer class is highly inspired by the Pinocchio version
    of the MeshCat visualizer
    https://github.com/stack-of-tasks/pinocchio/blob/b134b25f1409f5bf036105b996da2d29c1a66a12/bindings/python/pinocchio/visualize/meshcat_visualizer.py
    """

    def __init__(self, zmq_url=None):
        import meshcat
        
        if zmq_url is not None:
            print("Connecting to meshcat-server at zmq_url=" + zmq_url + ".")
          
        self.viewer = meshcat.Visualizer(zmq_url=zmq_url)
        self.traversal = dict()
        self.model = dict()
        self.link_pos = dict()

    def __is_mesh(self, geometry_object) -> bool:

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

    def __load_mesh(self, geometry_object):

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

    def __apply_transform(self, world_H_frame, solid_shape, viewer_name):
        world_H_geometry = (world_H_frame * solid_shape.getLink_H_geometry()).asHomogeneousTransform().toNumPy()
        scale = list(solid_shape.asExternalMesh().getScale().toNumPy().flatten())
        extended_scale = np.diag(np.concatenate((scale, [1.0])))
        world_H_geometry_scaled = np.array(world_H_geometry).dot(extended_scale)

        # Update viewer configuration.
        self.viewer[viewer_name].set_transform(world_H_geometry_scaled)

    def __model_exists(self, model_name):

        if model_name in self.model.keys():
            return True

        if model_name in self.traversal.keys():
            return True

        if model_name in self.link_pos.keys():
            return True

        return False

    def __add_model_geometry_to_viewer(self, model, model_geometry: idyn.ModelSolidShapes,
                                       model_name, color):
        import meshcat

        if not self.__model_exists(model_name):
            msg = "The model named: " +  model_name + " does not exist."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        # Solve forward kinematics
        joint_pos = idyn.VectorDynSize(self.model[model_name].getNrOfJoints())
        joint_pos.zero()
        idyn.ForwardPositionKinematics(self.model[model_name], self.traversal[model_name],
                                       idyn.Transform.Identity(), joint_pos,
                                       self.link_pos[model_name])

        link_solid_shapes = model_geometry.getLinkSolidShapes()

        for link_index in range(0, self.model[model_name].getNrOfLinks()):

            world_H_frame = self.link_pos[model_name](link_index)
            link_name = self.model[model_name].getLinkName(link_index)

            is_mesh = False
            for geom in range(0, len(link_solid_shapes[link_index])):
                solid_shape = model_geometry.getLinkSolidShapes()[link_index][geom]
                if self.__is_mesh(solid_shape):
                    obj = self.__load_mesh(solid_shape)
                    is_mesh = True
                else:

                    msg = "The geometry object named \"" \
                          + solid_shape.getName() \
                          + "\" is not supported by iDynTree/MeshCat for visualization."
                    warnings.warn(msg, category=UserWarning, stacklevel=2)
                    continue

                if obj is None:
                    msg = "The geometry object named " + solid_shape.asExternalMesh().getName() + " is not valid."
                    warnings.warn(msg, category=UserWarning, stacklevel=2)
                    continue

                viewer_name = model_name + "/" + link_name + "/" + solid_shape.asExternalMesh().getName()

                if isinstance(obj, meshcat.geometry.Object):
                    self.viewer[viewer_name].set_object(obj)
                elif isinstance(obj, meshcat.geometry.Geometry):
                    material = meshcat.geometry.MeshPhongMaterial()
                    # Set material color from URDF, converting for triplet of doubles to a single int.
                    if color is None:
                        mesh_color = solid_shape.getMaterial().color()
                    else:
                        mesh_color = color

                    material.color = int(mesh_color[0] * 255) * 256 ** 2 + \
                                     int(mesh_color[1] * 255) * 256 + \
                                     int(mesh_color[2] * 255)

                    # Add transparency, if needed.
                    if float(mesh_color[3]) != 1.0:
                        material.transparent = True
                        material.opacity = float(mesh_color[3])

                    self.viewer[viewer_name].set_object(obj, material)

                    if is_mesh:
                        self.__apply_transform(world_H_frame, solid_shape, viewer_name)

    def display(self, base_position, base_rotation, joint_value, model_name='iDynTree'):
        """Display the robot at given configuration."""

        if not self.__model_exists(model_name):
            msg = "The model named: " +  model_name + " does not exist."
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
        idyn.ForwardPositionKinematics(self.model[model_name], self.traversal[model_name], base_pose_idyn,
                                       joint_pos_idyn, self.link_pos[model_name])

        # Update the visual shapes
        model_geometry = self.model[model_name].visualSolidShapes()
        link_solid_shapes = model_geometry.getLinkSolidShapes()

        for link_index in range(0, self.model[model_name].getNrOfLinks()):

            link_name = self.model[model_name].getLinkName(link_index)
            for geom in range(0, len(link_solid_shapes[link_index])):
                solid_shape = model_geometry.getLinkSolidShapes()[link_index][geom]
                if self.__is_mesh(solid_shape):
                    viewer_name = model_name + "/" + link_name + "/" + solid_shape.asExternalMesh().getName()
                    self.__apply_transform(self.link_pos[model_name](link_index), solid_shape, viewer_name)

    def open(self):
        self.viewer.open()

    def jupyter_cell(self):
        return self.viewer.jupyter_cell()

    def set_model_from_file(self, model_path: str, considered_joints=None, model_name='iDynTree'):

        if self.__model_exists(model_name):
            msg = "The model named: " +  model_name + " already exists."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return


        model_loader = idyn.ModelLoader()
        if considered_joints is None:
            ok = model_loader.loadModelFromFile(model_path)
        else:
            considered_joints_idyn = idyn.StringVector()
            for joint in considered_joints:
                considered_joints_idyn.push_back(joint)

            ok = model_loader.loadReducedModelFromFile(model_path, considered_joints_idyn)

        if not ok:
            msg = "Unable to load the model named: " + model_name + " from the file: " + model_path + "."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        self.model[model_name] = model_loader.model().copy()
        self.traversal[model_name] = idyn.Traversal()
        self.link_pos[model_name] = idyn.LinkPositions()

        self.model[model_name].computeFullTreeTraversal(self.traversal[model_name])
        self.link_pos[model_name].resize(self.model[model_name])

    def set_model(self, model: idyn.Model, model_name='iDynTree'):

        if self.__model_exists(model_name):
            msg = "The model named: " + model_name + " already exists."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        self.model[model_name] = model.copy()
        self.traversal[model_name] = idyn.Traversal()
        self.link_pos[model_name] = idyn.LinkPositions()

        self.model[model_name].computeFullTreeTraversal(self.traversal[model_name])
        self.link_pos[model_name].resize(self.model[model_name])

    def load_model(self, model_name='iDynTree', color=None):
        self.__add_model_geometry_to_viewer(self.model,
                                            self.model[model_name].visualSolidShapes(),
                                            model_name,
                                            color)
