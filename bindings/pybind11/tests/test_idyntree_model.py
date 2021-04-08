"""Tests for idyntree-model Python bindings."""
import itertools
import unittest

import idyntree.pybind as iDynTree


class IDynTreeMaterialTest(unittest.TestCase):

  def test_creation(self):
    self.assertIsNotNone(iDynTree.Material())

  def test_creation_with_name(self):
    material = iDynTree.Material("a_material")
    self.assertEqual(material.name, "a_material")

  def test_color(self):
    material = iDynTree.Material()
    color = iDynTree.Vector4()
    color[0] = 1.0
    color[1] = 0.0
    color[2] = 0.0
    color[3] = 1.0
    material.color = color
    self.assertEqual(list(material.color), list(color))

  def test_texture(self):
    material = iDynTree.Material()
    material.texture = "a_texture"
    self.assertEqual(material.texture, "a_texture")


class IDynTreeSolidShapesTest(unittest.TestCase):

  def test_name(self):
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_name"
    self.assertEqual(mesh.name, "a_name")

  def test_transform(self):
    position = iDynTree.Position(1, 2, 3)
    rotation = iDynTree.Rotation(0, 0, 1,
                                 1, 0, 0,
                                 0, 1, 0)
    transform = iDynTree.Transform(rotation, position)
    mesh = iDynTree.ExternalMesh()
    mesh.link_H_geometry = transform

    self.assertEqual(list(mesh.link_H_geometry.position), list(position))
    for r, c in itertools.product(range(3), range(3)):
      self.assertEqual(mesh.link_H_geometry.rotation[r, c], rotation[r, c])

  def test_material(self):
    mesh = iDynTree.ExternalMesh()
    mesh.material = iDynTree.Material("a_material")
    self.assertEqual(mesh.material.name, "a_material")

  def test_clone(self):
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_name"
    mesh_clone = mesh.clone()
    mesh_clone.name = "cloned_name"
    self.assertEqual(mesh_clone.name, "cloned_name")
    self.assertEqual(mesh.name, "a_name")

  def test_mesh_filename(self):
    mesh = iDynTree.ExternalMesh()
    mesh.filename = "a_filename"
    self.assertEqual(mesh.filename, "a_filename")

  def test_mesh_scale(self):
    mesh = iDynTree.ExternalMesh()
    scale = iDynTree.Vector3()
    scale[0] = 1
    scale[1] = 2
    scale[2] = 3
    mesh.scale = scale
    self.assertEqual(list(mesh.scale), list(scale))

  def test_sphere_radius(self):
    sphere = iDynTree.Sphere()
    sphere.radius = 3.14
    self.assertEqual(sphere.radius, 3.14)

  def test_cast_to_correct_object_is_ok(self):
    sphere = iDynTree.Sphere()
    sphere.radius = 3.14
    mesh = iDynTree.ExternalMesh()
    mesh.filename = "a_filename"
    to_sphere = sphere.as_sphere()
    self.assertIsNotNone(to_sphere)
    self.assertEqual(to_sphere.radius, 3.14)
    to_mesh = mesh.as_external_mesh()
    self.assertIsNotNone(to_mesh)
    self.assertEqual(to_mesh.filename, "a_filename")

  def test_cast_to_wrong_object_raises(self):
    sphere = iDynTree.Sphere()
    sphere.radius = 3.14
    mesh = iDynTree.ExternalMesh()
    mesh.filename = "a_filename"
    with self.assertRaises(ValueError):
      sphere.as_external_mesh()
    with self.assertRaises(ValueError):
      mesh.as_sphere()


class IDynTreeLinkTest(unittest.TestCase):

  def test_index(self):
    link = iDynTree.Link()
    link.set_index(5)
    self.assertEqual(link.get_index(), 5)

  def test_inertia(self):
    mass = 3.14
    position = iDynTree.Position(1, 2, 3)
    rotational_inertia = iDynTree.RotationalInertia()
    rotational_inertia[1, 1] = 1

    inertia = iDynTree.SpatialInertia(mass, position, rotational_inertia)
    link = iDynTree.Link()
    link.inertia = inertia
    self.assertEqual(link.inertia.get_mass(), mass)


class IDynTreeRevoluteJointTest(unittest.TestCase):

  def test_attached_links(self):
    joint = iDynTree.RevoluteJoint()
    joint.set_attached_links(1, 2)
    self.assertEqual(joint.get_first_attached_link(), 1)
    self.assertEqual(joint.get_second_attached_link(), 2)

  def test_rest_transform(self):
    position = iDynTree.Position(1, 2, 3)
    rotation = iDynTree.Rotation(0, 0, 1,
                                 1, 0, 0,
                                 0, 1, 0)
    transform = iDynTree.Transform(rotation, position)

    joint = iDynTree.RevoluteJoint()
    joint.set_attached_links(1, 2)
    joint.set_rest_transform(transform)
    joint_transform = joint.get_rest_transform(1, 2)
    self.assertEqual(list(joint_transform.position), list(position))
    for r, c in itertools.product(range(3), range(3)):
      self.assertEqual(joint_transform.rotation[r, c], rotation[r, c])


class IDynTreePrismaticJointTest(unittest.TestCase):

  def test_attached_links(self):
    joint = iDynTree.PrismaticJoint()
    joint.set_attached_links(1, 2)
    self.assertEqual(joint.get_first_attached_link(), 1)
    self.assertEqual(joint.get_second_attached_link(), 2)

  def test_rest_transform(self):
    position = iDynTree.Position(1, 2, 3)
    rotation = iDynTree.Rotation(0, 0, 1,
                                 1, 0, 0,
                                 0, 1, 0)
    transform = iDynTree.Transform(rotation, position)

    joint = iDynTree.PrismaticJoint()
    joint.set_attached_links(1, 2)
    joint.set_rest_transform(transform)
    joint_transform = joint.get_rest_transform(1, 2)
    self.assertEqual(list(joint_transform.position), list(position))
    for r, c in itertools.product(range(3), range(3)):
      self.assertEqual(joint_transform.rotation[r, c], rotation[r, c])


class IDynTreeFixedJointTest(unittest.TestCase):

  def test_attached_links(self):
    joint = iDynTree.FixedJoint()
    joint.set_attached_links(1, 2)
    self.assertEqual(joint.get_first_attached_link(), 1)
    self.assertEqual(joint.get_second_attached_link(), 2)

  def test_rest_transform(self):
    position = iDynTree.Position(1, 2, 3)
    rotation = iDynTree.Rotation(0, 0, 1,
                                 1, 0, 0,
                                 0, 1, 0)
    transform = iDynTree.Transform(rotation, position)

    joint = iDynTree.FixedJoint()
    joint.set_attached_links(1, 2)
    joint.set_rest_transform(transform)
    joint_transform = joint.get_rest_transform(1, 2)
    self.assertEqual(list(joint_transform.position), list(position))
    for r, c in itertools.product(range(3), range(3)):
      self.assertEqual(joint_transform.rotation[r, c], rotation[r, c])


class IDynTreeTraversalTest(unittest.TestCase):

  def test_empty_traversal(self):
    traversal = iDynTree.Traversal()
    self.assertEqual(traversal.get_nr_of_visited_links(), 0)


class IDynTreeModelSolidShapesTest(unittest.TestCase):

  def test_resize_with_size(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model_shapes.resize(2)
    self.assertEqual(len(model_shapes), 2)

  def test_resize_with_model(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model = iDynTree.Model()
    link1 = iDynTree.Link()
    link2 = iDynTree.Link()
    model.add_link("link1", link1)
    model.add_link("link2", link2)
    model_shapes.resize(model)
    self.assertEqual(len(model_shapes), model.get_nr_of_links())

  def test_add_mesh(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model_shapes.resize(2)
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_mesh"
    model_shapes[0].append(mesh)
    self.assertEqual(len(model_shapes[1]), 0)
    self.assertEqual(model_shapes[0, 0].name, "a_mesh")

  def test_clear_on_model_shapes(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model_shapes.resize(2)
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_mesh"
    model_shapes[0].append(mesh)
    self.assertEqual(len(model_shapes), 2)
    model_shapes.clear()
    self.assertEqual(len(model_shapes), 0)

  def test_clear_on_link_shapes(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model_shapes.resize(1)
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_mesh"
    model_shapes[0].append(mesh)
    self.assertEqual(len(model_shapes[0]), 1)
    model_shapes[0].clear()
    self.assertEqual(len(model_shapes), 1)
    self.assertEqual(len(model_shapes[0]), 0)

  def test_overwrite_shape_on_link_list(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model_shapes.resize(1)
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_mesh"
    model_shapes[0].append(mesh)
    # Now change the shape in [0][0]
    mesh.name = "a_different_mesh"
    model_shapes[0][0] = mesh
    self.assertEqual(model_shapes[0, 0].name, "a_different_mesh")

  def test_overwrite_shape_on_model_list(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model_shapes.resize(1)
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_mesh"
    model_shapes[0].append(mesh)
    # Now change the shape in [0][0]
    mesh.name = "a_different_mesh"
    model_shapes[0, 0] = mesh
    self.assertEqual(model_shapes[0, 0].name, "a_different_mesh")

  def test_link_iterator(self):
    model_shapes = iDynTree.ModelSolidShapes()
    model_shapes.resize(2)
    # Create one mesh for link 0.
    mesh = iDynTree.ExternalMesh()
    mesh.name = "mesh_l0"
    model_shapes[0].append(mesh)
    # Create two meshes for link 1.
    mesh = iDynTree.ExternalMesh()
    mesh.name = "mesh_l1_1"
    model_shapes[1].append(mesh)
    mesh = iDynTree.ExternalMesh()
    mesh.name = "mesh_l1_2"
    model_shapes[1].append(mesh)
    # Test that the iterator iterates once on the first link.
    i = 0
    for m in model_shapes[0]:
      i += 1
      self.assertTrue(m.name)
    self.assertEqual(i, 1)
    # Test that the iterator iterates twice on the first link.
    i = 0
    for m in model_shapes[1]:
      i += 1
      self.assertTrue(m.name)
    self.assertEqual(i, 2)


class IDynTreeModelTest(unittest.TestCase):

  def test_model_one_link_model(self):
    model = iDynTree.Model()
    link1 = iDynTree.Link()
    self.assertEqual(model.add_link("link1", link1), 0)
    self.assertEqual(model.get_nr_of_links(), 1)
    self.assertEqual(model.get_link_index("link1"), 0)
    self.assertEqual(model.get_link_name(0), "link1")
    self.assertIsNotNone(model.get_link(0))

  def test_model_default_base(self):
    model = iDynTree.Model()
    link1 = iDynTree.Link()
    link2 = iDynTree.Link()
    model.add_link("link1", link1)
    link2_idx = model.add_link("link2", link2)
    model.default_base_link = link2_idx
    self.assertEqual(model.default_base_link, link2_idx)

  def test_model_one_joint(self):
    model = iDynTree.Model()
    link1 = iDynTree.Link()
    link2 = iDynTree.Link()
    model.add_link("link1", link1)
    model.add_link("link2", link2)
    joint = iDynTree.RevoluteJoint()
    joint.set_attached_links(0, 1)
    self.assertEqual(model.add_joint("joint1", joint), 0)
    self.assertEqual(model.get_nr_of_joints(), 1)
    self.assertEqual(model.get_joint_index("joint1"), 0)
    self.assertEqual(model.get_joint_name(0), "joint1")
    self.assertIsNotNone(model.get_joint(0))

  def test_model_frame(self):
    position = iDynTree.Position(1, 2, 3)
    rotation = iDynTree.Rotation(0, 0, 1,
                                 1, 0, 0,
                                 0, 1, 0)
    transform = iDynTree.Transform(rotation, position)
    model = iDynTree.Model()
    link1 = iDynTree.Link()
    model.add_link("link1", link1)
    num_of_frames = model.get_nr_of_frames()
    self.assertTrue(
        model.add_additional_frame_to_link("link1", "frame1", transform))
    self.assertEqual(model.get_nr_of_frames(), num_of_frames + 1)
    frame_index = model.get_frame_index("frame1")
    self.assertGreaterEqual(frame_index, 0)
    self.assertEqual(model.get_frame_name(frame_index), "frame1")
    self.assertEqual(model.get_frame_link(frame_index), 0)
    frame_transform = model.get_frame_transform(frame_index)
    self.assertEqual(list(frame_transform.position), list(position))
    for r, c in itertools.product(range(3), range(3)):
      self.assertEqual(frame_transform.rotation[r, c], rotation[r, c])

  def test_compute_traversal(self):
    model = iDynTree.Model()
    link1 = iDynTree.Link()
    link2 = iDynTree.Link()
    model.add_link("link1", link1)
    model.add_link("link2", link2)
    joint = iDynTree.RevoluteJoint()
    joint.set_attached_links(0, 1)
    model.add_joint("joint1", joint)
    traversal = model.compute_full_tree_traversal(0)
    self.assertIsNotNone(traversal)
    self.assertEqual(traversal.get_nr_of_visited_links(), 2)

  def test_model_shapes(self):
    model = iDynTree.Model()
    link1 = iDynTree.Link()
    link2 = iDynTree.Link()
    model.add_link("link1", link1)
    model.add_link("link2", link2)
    self.assertEqual(len(model.visual_solid_shapes), model.get_nr_of_links())
    self.assertEqual(len(model.collision_solid_shapes), model.get_nr_of_links())
    # Append a shape to test that the objects are writable.
    mesh = iDynTree.ExternalMesh()
    mesh.name = "a_mesh"
    old_len = len(model.visual_solid_shapes[0])
    model.visual_solid_shapes[0].append(mesh)
    self.assertEqual(len(model.visual_solid_shapes[0]), old_len + 1)


if __name__ == "__main__":
  unittest.main()
