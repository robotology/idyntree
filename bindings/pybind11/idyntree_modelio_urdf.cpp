#include "idyntree_modelio_urdf.h"

#include "error_utilities.h"

#include <iDynTree/Model.h>
#include <iDynTree/ModelExporter.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Sensors.h>
#include <string>
#include <vector>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace iDynTree
{
namespace bindings
{
namespace
{

class ExporterHelper
{
public:
    static std::string exportAsString(ModelExporter* this_)
    {
        std::string output_model;
        if (!this_->exportModelToString(output_model))
        {
            raisePythonException(PyExc_ValueError, "Failed to export model.");
        }
        return output_model;
    }

    static void init(ModelExporter* this_,
                     const Model& model,
                     const SensorsList& sensors = SensorsList(),
                     const ModelExporterOptions options = ModelExporterOptions())
    {
        if (!this_->init(model, sensors, options))
        {
            raisePythonException(PyExc_ValueError, "Failed to initialize model.");
        }
    }
};

namespace py = ::pybind11;

} // namespace

void iDynTreeModelIoUrdfBindings(pybind11::module& module)
{
    py::class_<ModelExporterOptions>(module, "ModelExporterOptions")
        .def(py::init())
        .def_readwrite("base_link", &ModelExporterOptions::baseLink)
        .def_readwrite("robot_exported_name", &ModelExporterOptions::robotExportedName)
        .def_readwrite("export_first_base_link_additional_frame_as_fake_urdfbase",
                       &ModelExporterOptions::exportFirstBaseLinkAdditionalFrameAsFakeURDFBase)
        .def_readwrite("export_spherical_joints_as_three_revolute_joints",
                       &ModelExporterOptions::exportSphericalJointsAsThreeRevoluteJoints);

    py::class_<ModelParserOptions>(module, "ModelParserOptions")
        .def(py::init())
        .def_readwrite("add_sensor_frames_as_additional_frames",
                       &ModelParserOptions::addSensorFramesAsAdditionalFrames)
        .def_readwrite("original_filename", &ModelParserOptions::originalFilename)
        .def_readwrite("convert_three_revolute_joints_to_spherical_joint",
                       &ModelParserOptions::convertThreeRevoluteJointsToSphericalJoint)
        .def_readwrite("spherical_joint_zero_mass_tolerance",
                       &ModelParserOptions::sphericalJointZeroMassTolerance)
        .def_readwrite("spherical_joint_orthogonality_tolerance",
                       &ModelParserOptions::sphericalJointOrthogonalityTolerance)
        .def_readwrite("spherical_joint_intersection_tolerance",
                       &ModelParserOptions::sphericalJointIntersectionTolerance);

    py::class_<ModelExporter>(module, "ModelExporter")
        .def(py::init())
        .def("init",
             &ExporterHelper::init,
             py::arg("model"),
             py::arg("sensors") = SensorsList(),
             py::arg("options") = ModelExporterOptions())
        .def_property_readonly("model", &ModelExporter::model)
        .def_property_readonly("sensors", &ModelExporter::sensors)
        .def_property_readonly("options", &ModelExporter::exportingOptions)
        .def("export_model_to_string", &ExporterHelper::exportAsString);

    py::class_<ModelLoader>(module, "ModelLoader")
        .def(py::init())
        .def("load_model_from_file",
             [](ModelLoader* this_, const std::string& filename) {
                 return this_->loadModelFromFile(filename);
             })
        .def("load_reduced_model_from_full_model",
             [](ModelLoader* this_, const Model& model, const std::vector<std::string>& joints) {
                 return this_->loadReducedModelFromFullModel(model, joints);
             })
        .def_property_readonly("model", &ModelLoader::model)
        .def_property("parsing_options",
                      [](ModelLoader& loader) { return loader.parsingOptions(); },
                      &ModelLoader::setParsingOptions);
}

} // namespace bindings
} // namespace iDynTree
