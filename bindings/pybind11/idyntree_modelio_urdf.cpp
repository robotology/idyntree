#include "idyntree_modelio_urdf.h"

#include "error_utilities.h"

#include <string>
#include <iDynTree/ModelIO/ModelExporter.h>
#include <iDynTree/Sensors/Sensors.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


namespace iDynTree {
namespace bindings {
namespace {

class ExporterHelper {
 public:
  static std::string exportAsString(ModelExporter* this_) {
    std::string output_model;
    if (!this_->exportModelToString(output_model)) {
      raisePythonException(PyExc_ValueError, "Failed to export model.");
    }
    return output_model;
  }

  static void init(
      ModelExporter* this_, const Model& model,
      const SensorsList& sensors = SensorsList(),
      const ModelExporterOptions options = ModelExporterOptions()) {
    if (!this_->init(model, sensors, options)) {
      raisePythonException(PyExc_ValueError, "Failed to initialize model.");
    }
  }
};

namespace py = ::pybind11;

}  // namespace

void iDynTreeModelIoUrdfBindings(pybind11::module& module) {
  py::class_<ModelExporterOptions>(module, "ModelExporterOptions")
      .def(py::init())
      .def_readwrite("base_link", &ModelExporterOptions::baseLink)
      .def_readwrite("robot_exported_name",
                     &ModelExporterOptions::robotExportedName)
      .def_readwrite("export_first_base_link_additional_frame_as_fake_urdfbase",
                     &ModelExporterOptions::
                         exportFirstBaseLinkAdditionalFrameAsFakeURDFBase);

  py::class_<ModelExporter>(module, "ModelExporter")
      .def(py::init())
      .def("init", &ExporterHelper::init, py::arg("model"),
           py::arg("sensors") = SensorsList(),
           py::arg("options") = ModelExporterOptions())
      .def_property_readonly("model", &ModelExporter::model)
      .def_property_readonly("sensors", &ModelExporter::sensors)
      .def_property_readonly("options", &ModelExporter::exportingOptions)
      .def("export_model_to_string", &ExporterHelper::exportAsString);
}

}  // namespace bindings
}  // namespace iDynTree
