"""Tests for idyntree-model-io-urdf Python bindings."""
import os
import sys
import tempfile
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]
PYBIND_DIR = ROOT / "build" / "bindings" / "pybind11" / "idyntree"
sys.path.insert(0, str(PYBIND_DIR))

import pybind as iDynTree

SPHERICAL_COMPATIBLE_URDF = """
<robot name="test_robot">
  <link name="link1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <link name="link2">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>
  <link name="link3">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
  </link>
  <link name="link4">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="joint2" type="continuous">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="joint3" type="continuous">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""


class IDynTreeModelIoUrdfTest(unittest.TestCase):

  def test_exporter_options(self):
    options = iDynTree.ModelExporterOptions()
    options.base_link = "a_base_link"
    options.robot_exported_name = "robot_name"
    options.export_first_base_link_additional_frame_as_fake_urdfbase = True
    self.assertEqual(options.base_link, "a_base_link")
    self.assertEqual(options.robot_exported_name, "robot_name")
    self.assertTrue(
        options.export_first_base_link_additional_frame_as_fake_urdfbase)
    options.export_first_base_link_additional_frame_as_fake_urdfbase = False
    self.assertFalse(
        options.export_first_base_link_additional_frame_as_fake_urdfbase)

  def test_exporter(self):
    exporter = iDynTree.ModelExporter()
    model = iDynTree.Model()
    link = iDynTree.Link()
    model.add_link("a_link", link)
    exporter.init(model)
    self.assertTrue(exporter.export_model_to_string())
    self.assertEqual(exporter.model.get_nr_of_links(), model.get_nr_of_links())

  def test_custom_options(self):
    options = iDynTree.ModelExporterOptions()
    options.robot_exported_name = "robot_name"
    exporter = iDynTree.ModelExporter()
    model = iDynTree.Model()
    link = iDynTree.Link()
    model.add_link("a_link", link)
    exporter.init(model, options=options)
    self.assertEqual(exporter.options.robot_exported_name, "robot_name")

  def test_custom_sensors(self):
    exporter = iDynTree.ModelExporter()
    model = iDynTree.Model()
    link = iDynTree.Link()
    model.add_link("a_link", link)
    sensors = iDynTree.SensorsList()
    sensor = iDynTree.AccelerometerSensor()
    sensor.name = "a_name"
    sensor.parent_link_index = 1
    sensors.add_sensor(sensor)
    exporter.init(model, sensors=sensors)
    self.assertEqual(
        exporter.sensors.get_nr_of_sensors(iDynTree.ACCELEROMETER), 1)

  def test_importer_without_conversion_option(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      urdf_filename = os.path.join(tmpdir, 'model.urdf')
      with open(urdf_filename, 'w') as f:
        f.write(SPHERICAL_COMPATIBLE_URDF)

      importer = iDynTree.ModelLoader()
      importer.load_model_from_file(urdf_filename)
      self.assertEqual(3, importer.model.get_nr_of_joints())
      self.assertIsInstance(importer.model.get_joint(0), iDynTree.RevoluteJoint)

  def test_importer_converts_to_spherical_joint(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      urdf_filename = os.path.join(tmpdir, 'model.urdf')
      with open(urdf_filename, 'w') as f:
        f.write(SPHERICAL_COMPATIBLE_URDF)

      importer = iDynTree.ModelLoader()
      options = iDynTree.ModelParserOptions()
      options.convert_three_revolute_joints_to_spherical_joint = True
      importer.parsing_options = options
      importer.load_model_from_file(urdf_filename)

      self.assertEqual(1, importer.model.get_nr_of_joints())
      self.assertEqual(2, importer.model.get_nr_of_links())
      self.assertIsInstance(importer.model.get_joint(0), iDynTree.SphericalJoint)

  def test_importer(self):
    model = (r'<robot name="test_robot">'
             r'<link name="link1" />'
             r'<link name="link2" />'
             r'<link name="link3" />'
             r'<link name="link4" />'
             r'<joint name="joint1" type="continuous">'
             r'<parent link="link1"/>'
             r'<child link="link2"/>'
             r'</joint>'
             r'<joint name="joint2" type="continuous">'
             r'<parent link="link1"/>'
             r'<child link="link3"/>'
             r'</joint>'
             r'<joint name="joint3" type="continuous">'
             r'<parent link="link3"/>'
             r'<child link="link4"/>'
             r'</joint>'
             r'</robot>')

    tmpdir = tempfile.TemporaryDirectory()
    urdf_filename = os.path.join(tmpdir.name, 'model.urdf')
    with open(urdf_filename, 'w') as f:
      f.write(model)

    importer = iDynTree.ModelLoader()
    importer.load_model_from_file(urdf_filename)
    self.assertEqual(4, importer.model.get_nr_of_links())
    self.assertEqual(3, importer.model.get_nr_of_joints())

  def test_importer_reduced_model(self):
    model = (r'<robot name="test_robot">'
             r'<link name="link1" />'
             r'<link name="link2" />'
             r'<link name="link3" />'
             r'<link name="link4" />'
             r'<joint name="joint1" type="continuous">'
             r'<parent link="link1"/>'
             r'<child link="link2"/>'
             r'</joint>'
             r'<joint name="joint2" type="continuous">'
             r'<parent link="link1"/>'
             r'<child link="link3"/>'
             r'</joint>'
             r'<joint name="joint3" type="continuous">'
             r'<parent link="link3"/>'
             r'<child link="link4"/>'
             r'</joint>'
             r'</robot>')

    tmpdir = tempfile.TemporaryDirectory()
    urdf_filename = os.path.join(tmpdir.name, 'model.urdf')
    with open(urdf_filename, 'w') as f:
      f.write(model)

    importer = iDynTree.ModelLoader()
    importer.load_model_from_file(urdf_filename)
    importer.load_reduced_model_from_full_model(importer.model,
                                                ('joint1', 'joint3'))
    # The reduced model will have the 2 joints we specified, and one less body
    # as the missing joint made two bodies being merged together.
    self.assertEqual(3, importer.model.get_nr_of_links())
    self.assertEqual(2, importer.model.get_nr_of_joints())
    self.assertEqual('joint1', importer.model.get_joint_name(0))
    self.assertEqual('joint3', importer.model.get_joint_name(1))


if __name__ == "__main__":
  unittest.main()
