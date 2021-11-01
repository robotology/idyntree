/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/ModelIO/ModelExporter.h"

#include "URDFModelExport.h"

#include <string>
#include <vector>

namespace iDynTree
{

ModelExporterOptions::ModelExporterOptions()
: baseLink(""),
    exportFirstBaseLinkAdditionalFrameAsFakeURDFBase(true),
    robotExportedName("iDynTreeURDFModelExportModelName") {}


class ModelExporter::Pimpl {
public:
    Model m_model;
    SensorsList m_sensors;
    bool m_isModelValid;
    ModelExporterOptions m_options;

    bool setModelAndSensors(const Model& _model, const SensorsList& _sensors);
};

bool ModelExporter::Pimpl::setModelAndSensors(const Model& _model,
                                                           const SensorsList& _sensors)
{

    m_model = _model;
    m_sensors = _sensors;

    m_isModelValid = true;

    return true;
}

ModelExporter::ModelExporter()
: m_pimpl(new Pimpl())
{
    m_pimpl->m_isModelValid = false;
}

ModelExporter::~ModelExporter() {}

const Model& ModelExporter::model()
{
    return m_pimpl->m_model;
}

const SensorsList& ModelExporter::sensors()
{
    return m_pimpl->m_sensors;
}

bool ModelExporter::isValid()
{
    return m_pimpl->m_isModelValid;
}

const ModelExporterOptions& ModelExporter::exportingOptions() const
{
    return m_pimpl->m_options;
}

void ModelExporter::setExportingOptions(const ModelExporterOptions& options)
{
    m_pimpl->m_options = options;
}

bool ModelExporter::init(const Model& model,
                         const SensorsList& sensors,
                         const ModelExporterOptions options)
{
    m_pimpl->m_options = options;
    return m_pimpl->setModelAndSensors(model, sensors);
}

bool ModelExporter::exportModelToString(std::string & modelString, const std::string filetype)
{
    if (filetype != "urdf") {
        std::stringstream error_msg;
        error_msg << "Filetype " << filetype << " not supported. Only urdf format is currently supported.";
        reportError("ModelExporter", "exportModelToString", error_msg.str().c_str());
        return false;
    }

    return URDFStringFromModel(m_pimpl->m_model, modelString, m_pimpl->m_options);
}

/**
 * Export the model of the robot to an external file.
 *
 * @param filename path to the file to export
 * @param filetype type of the file to load, currently supporting only urdf type.
 *
 */
bool ModelExporter::exportModelToFile(const std::string & filename, const std::string filetype)
{
    if (filetype != "urdf") {
        std::stringstream error_msg;
        error_msg << "Filetype " << filetype << " not supported. Only urdf format is currently supported.";
        reportError("ModelExporter", "exportModelToFile", error_msg.str().c_str());
        return false;
    }

    return URDFFromModel(m_pimpl->m_model, filename, m_pimpl->m_options);
}

}
