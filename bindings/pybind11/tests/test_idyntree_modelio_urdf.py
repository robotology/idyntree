"""Tests for idyntree-model-io-urdf Python bindings."""
import os
import tempfile
import unittest

import idyntree.pybind as iDynTree


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
