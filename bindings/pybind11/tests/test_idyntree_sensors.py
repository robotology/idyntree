"""Tests for idyntree-sensors Python bindings."""
import itertools
import unittest

import idyntree.pybind as iDynTree


class IDynTreeSensorsTest(unittest.TestCase):

  def test_six_axis_ft_sensor(self):
    sensor = iDynTree.SixAxisForceTorqueSensor()
    self.assertEqual(sensor.sensor_type, iDynTree.SIX_AXIS_FORCE_TORQUE)

  def test_accelerometer_sensor(self):
    sensor = iDynTree.AccelerometerSensor()
    self.assertEqual(sensor.sensor_type, iDynTree.ACCELEROMETER)

  def test_joint_sensor_name(self):
    sensor = iDynTree.SixAxisForceTorqueSensor()
    sensor.name = "a_sensor"
    self.assertEqual(sensor.name, "a_sensor")

  def test_joint_sensor_joint(self):
    sensor = iDynTree.SixAxisForceTorqueSensor()
    sensor.parent_joint = "a_joint"
    self.assertEqual(sensor.parent_joint, "a_joint")

  def test_joint_sensor_joint_index(self):
    sensor = iDynTree.SixAxisForceTorqueSensor()
    sensor.parent_joint_index = 1
    self.assertEqual(sensor.parent_joint_index, 1)

  def test_link_sensor_name(self):
    sensor = iDynTree.AccelerometerSensor()
    sensor.name = "a_sensor"
    self.assertEqual(sensor.name, "a_sensor")

  def test_link_sensor_link(self):
    sensor = iDynTree.AccelerometerSensor()
    sensor.parent_link = "a_link"
    self.assertEqual(sensor.parent_link, "a_link")

  def test_link_sensor_link_index(self):
    sensor = iDynTree.AccelerometerSensor()
    sensor.parent_link_index = 1
    self.assertEqual(sensor.parent_link_index, 1)

  def test_link_sensor_transform(self):
    sensor = iDynTree.AccelerometerSensor()
    position = iDynTree.Position(1, 2, 3)
    rotation = iDynTree.Rotation(0, 0, 1,
                                 1, 0, 0,
                                 0, 1, 0)
    sensor.link_sensor_transform = iDynTree.Transform(rotation, position)

    # Assert the content is the same.
    for r, c in itertools.product(range(3), range(3)):
      self.assertEqual(
          sensor.link_sensor_transform.rotation[r, c], rotation[r, c])
    for i in range(3):
      self.assertEqual(sensor.link_sensor_transform.position[i], position[i])

  def test_sensors_list_creation(self):
    self.assertIsNotNone(iDynTree.SensorsList())

  def test_sensors_list_add_sensors(self):
    sensors = iDynTree.SensorsList()
    sensor = iDynTree.AccelerometerSensor()
    sensor.name = "a_name"
    sensor.parent_link_index = 1
    sensors.add_sensor(sensor)
    self.assertEqual(sensors.get_nr_of_sensors(iDynTree.ACCELEROMETER), 1)
    self.assertEqual(sensors.get_nr_of_sensors(iDynTree.GYROSCOPE), 0)


if __name__ == "__main__":
  unittest.main()
