#include "idyntree_sensors.h"
#include "error_utilities.h"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iDynTree/AccelerometerSensor.h>
#include <iDynTree/Sensors.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>

namespace iDynTree {
namespace bindings {
namespace {

class SensorsListHelper {
 public:
   static void addSensor(SensorsList* this_, const Sensor& sensor) {
     if (this_->addSensor(sensor)) {
       raisePythonException(PyExc_ValueError, "Failed to add sensor.");
     }
   }
};

namespace py = ::pybind11;

}  // namespace

void iDynTreeSensorsBindings(pybind11::module& module) {
  py::enum_<SensorType>(module, "SensorType")
      .value("SIX_AXIS_FORCE_TORQUE", SIX_AXIS_FORCE_TORQUE)
      .value("ACCELEROMETER", ACCELEROMETER)
      .value("GYROSCOPE", GYROSCOPE)
      .value("THREE_AXIS_ANGULAR_ACCELEROMETER",
             THREE_AXIS_ANGULAR_ACCELEROMETER)
      .value("THREE_AXIS_FORCE_TORQUE_CONTACT", THREE_AXIS_FORCE_TORQUE_CONTACT)
      .export_values();

  py::class_<Sensor>(module, "_Sensor")
      .def_property("name", &Sensor::getName, &Sensor::setName)
      .def_property_readonly("sensor_type", &Sensor::getSensorType);

  py::class_<LinkSensor, Sensor>(module, "_LinkSensor")
      .def_property("parent_link", &LinkSensor::getParentLink,
                    &LinkSensor::setParentLink)
      .def_property("parent_link_index", &LinkSensor::getParentLinkIndex,
                    &LinkSensor::setParentLinkIndex)
      .def_property("link_sensor_transform",
                    &LinkSensor::getLinkSensorTransform,
                    &LinkSensor::setLinkSensorTransform);

  py::class_<AccelerometerSensor, LinkSensor>(module, "AccelerometerSensor")
      .def(py::init());

  py::class_<JointSensor, Sensor>(module, "_JointSensor")
      .def_property("parent_joint", &JointSensor::getParentJoint,
                    &JointSensor::setParentJoint)
      .def_property("parent_joint_index", &JointSensor::getParentJointIndex,
                    &JointSensor::setParentJointIndex);

  py::class_<SixAxisForceTorqueSensor, JointSensor>(module,
                                                    "SixAxisForceTorqueSensor")
      .def(py::init());

  py::class_<SensorsList>(module, "SensorsList")
      .def(py::init())
      .def("add_sensor", &SensorsListHelper::addSensor)
      .def("get_nr_of_sensors", &SensorsList::getNrOfSensors);
}

}  // namespace bindings
}  // namespace iDynTree
