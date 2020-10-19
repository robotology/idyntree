"""Tests for idyntree-model-io-urdf Python bindings."""
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


if __name__ == "__main__":
  unittest.main()
