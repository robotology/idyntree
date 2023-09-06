"""Tests for idyntree-high-level Python bindings."""
import itertools
import math
import unittest

import idyntree.pybind as iDynTree
import numpy as np


class IDynTreeHighLevelTest(unittest.TestCase):

  def _transform_equality_func(self,
                               expected: iDynTree.Transform,
                               other: iDynTree.Transform,
                               msg: str = None):
    equals = True
    error_msg = [msg] if msg else [""]
    for r, c in itertools.product(range(3), range(3)):
      check = math.isclose(expected.rotation[r, c], other.rotation[r, c])
      if not check:
        equals = False
        error_msg.append(
            f"Rotation [{r},{c}]: expected {expected.rotation[r, c]} "
            f"actual {other.rotation[r, c]}")
    for i in range(3):
      check = math.isclose(expected.position[i], other.position[i])
      if not check:
        equals = False
        error_msg.append(f"Position [{i}]: expected {expected.position[i]} "
                         f"actual {other.position[i]}")
    if not equals:
      raise self.failureException("\n".join(error_msg))

  def setUp(self):
    super().setUp()
    self.addTypeEqualityFunc(iDynTree.Transform, self._transform_equality_func)

    model = iDynTree.Model()
    # 3 links.
    link1 = iDynTree.Link()
    link2 = iDynTree.Link()
    link3 = iDynTree.Link()
    link1_idx = model.add_link("link1", link1)
    link2_idx = model.add_link("link2", link2)
    link3_idx = model.add_link("link3", link3)
    self.assertGreaterEqual(link1_idx, 0)
    self.assertGreaterEqual(link2_idx, 0)
    self.assertGreaterEqual(link3_idx, 0)

    # 2 joints. Axis is always Z.
    direction = iDynTree.Direction(0, 0, 1)
    axis = iDynTree.Axis(direction, iDynTree.Position.Zero())

    joint = iDynTree.RevoluteJoint()
    joint.set_attached_links(link1_idx, link2_idx)
    position = iDynTree.Position(0, 1, 0)
    rotation = iDynTree.Rotation(0, 0, 1, 1, 0, 0, 0, 1, 0)
    self._joint1_transform = iDynTree.Transform(rotation, position)
    joint.set_rest_transform(self._joint1_transform)
    joint.set_axis(axis, link2_idx, link1_idx)
    self._joint1_idx = model.add_joint("joint1", joint)
    self.assertGreaterEqual(self._joint1_idx, 0)

    joint = iDynTree.RevoluteJoint()
    joint.set_attached_links(link2_idx, link3_idx)
    position = iDynTree.Position(0, 1, 0)
    rotation = iDynTree.Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)
    self._joint2_transform = iDynTree.Transform(rotation, position)
    joint.set_rest_transform(self._joint2_transform)
    joint.set_axis(axis, link3_idx, link2_idx)
    self._joint2_idx = model.add_joint("joint2", joint)
    self.assertGreaterEqual(self._joint2_idx, 0)

    # 2 frames.
    position = iDynTree.Position(1, 0.2, -1.5)
    rotation = iDynTree.Rotation(0, 0, 1, 1, 0, 0, 0, 1, 0)
    self._base_frame_transform = iDynTree.Transform(rotation, position)
    model.add_additional_frame_to_link("link1", "base_frame",
                                       self._base_frame_transform)

    position = iDynTree.Position(-0.1, -0.5, 1.1)
    rotation = iDynTree.Rotation(1, 0, 0, 0, -1, 0, 0, 0, -1)
    self._ee_frame_transform = iDynTree.Transform(rotation, position)
    model.add_additional_frame_to_link("link3", "ee_frame",
                                       self._ee_frame_transform)

    self._kin_dyn = iDynTree.KinDynComputations()
    self.assertTrue(self._kin_dyn.load_robot_model(model))

  def test_model_info(self):
    self.assertEqual(2, self._kin_dyn.get_nr_of_degrees_of_freedom())
    self.assertEqual(3, self._kin_dyn.get_nr_of_links())
    # 3 links + 2 explicit frames.
    self.assertEqual(5, self._kin_dyn.get_nr_of_frames())

  def test_floating_base(self):
    # The default should be the first link as there are no reason to change it
    # while generating the traversal.
    self.assertEqual("link1", self._kin_dyn.get_floating_base())

    # Now set a new floating base.
    self.assertTrue(self._kin_dyn.set_floating_base("link3"))
    self.assertEqual("link3", self._kin_dyn.get_floating_base())

  def test_relative_transform(self):
    positions = iDynTree.VectorDynSize(
        self._kin_dyn.get_nr_of_degrees_of_freedom())
    positions.set_zero()

    velocities = iDynTree.VectorDynSize(
        self._kin_dyn.get_nr_of_degrees_of_freedom())
    velocities.set_zero()
    gravity = iDynTree.Vector3()
    gravity.set_zero()
    gravity[2] = -9.81

    # Set a robot state. Use a fixed base state.
    # Set an arbitrary configuration for joint2
    positions[self._joint2_idx] = np.deg2rad(45)
    self._kin_dyn.set_fixed_base_robot_state(positions, velocities, gravity)
    transform = self._kin_dyn.get_relative_transform_for_frames_named(
        "ee_frame", "link2")
    # Construct the expected transform.
    # ee_frame_H_link2 = (link2_H_link3^- * H_joint2 * link3_H_ee_frame) ^-1.
    joint2_var_transform = iDynTree.Transform(
        iDynTree.Rotation.RotZ(positions[self._joint2_idx]),
        iDynTree.Position.Zero())
    expected_transform = (self._joint2_transform * joint2_var_transform *
                          self._ee_frame_transform).inverse()

    self.assertEqual(expected_transform, transform)

  def test_absolute_transform(self):
    positions = iDynTree.VectorDynSize(
        self._kin_dyn.get_nr_of_degrees_of_freedom())
    positions.set_zero()

    velocities = iDynTree.VectorDynSize(
        self._kin_dyn.get_nr_of_degrees_of_freedom())
    velocities.set_zero()
    gravity = iDynTree.Vector3()
    gravity.set_zero()
    gravity[2] = -9.81

    # Arbitrary floating base world transform.
    floating_base_transform = iDynTree.Transform(
        iDynTree.Rotation(0.3224500, -0.9092497, 0.2632318, -0.9453749,
                          -0.3234002, 0.0409703, 0.0478770, -0.2620636,
                          -0.9638622), iDynTree.Position(1, 2, 3))

    base_twist = iDynTree.Twist([0, 0, 0], [0, 0, 0])

    # Set a robot state.
    # Set an arbitrary configuration for joint2
    positions[self._joint2_idx] = np.deg2rad(45)
    self._kin_dyn.set_robot_state(floating_base_transform, positions,
                                  base_twist, velocities, gravity)
    transform = self._kin_dyn.get_world_transform_for_frame_named("ee_frame")
    # Construct the expected transform.
    # world_H_ee_frame = (floating_base_transform * link1_H_link2 *
    #                     link2_H_link3^- * H_joint2 * link3_H_ee_frame).
    joint2_var_transform = iDynTree.Transform(
        iDynTree.Rotation.RotZ(positions[self._joint2_idx]),
        iDynTree.Position.Zero())
    expected_transform = (
        floating_base_transform * self._joint1_transform *
        self._joint2_transform * joint2_var_transform *
        self._ee_frame_transform)

    self.assertEqual(expected_transform, transform)


if __name__ == "__main__":
  unittest.main()
