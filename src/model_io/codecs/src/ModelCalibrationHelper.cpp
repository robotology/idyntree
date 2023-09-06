// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ModelCalibrationHelper.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelExporter.h>

#include <string>
#include <vector>

namespace iDynTree
{
    class ModelCalibrationHelper::ModelCalibrationHelperPimpl {
    public:
        ModelLoader modelLoader;
        ModelExporter modelExporter;

        ModelCalibrationHelperPimpl() {}
    };

    ModelCalibrationHelper::ModelCalibrationHelper()
    : m_pimpl(new ModelCalibrationHelperPimpl())
    {
    }

    ModelCalibrationHelper::~ModelCalibrationHelper() {}

    const Model& ModelCalibrationHelper::model()
    {
        return m_pimpl->modelLoader.model();
    }

    const SensorsList& ModelCalibrationHelper::sensors()
    {
        return m_pimpl->modelLoader.sensors();
    }

    bool ModelCalibrationHelper::isValid()
    {
        return m_pimpl->modelLoader.isValid();
    }

    bool ModelCalibrationHelper::loadModelFromString(const std::string& xmlString,
                                                     const std::string& filetype)
    {
        return m_pimpl->modelLoader.loadModelFromString(xmlString, filetype);
    }

    bool ModelCalibrationHelper::loadModelFromFile(const std::string& filename,
                                        const std::string& filetype)
    {
        return m_pimpl->modelLoader.loadModelFromFile(filename, filetype);
    }

    bool ModelCalibrationHelper::updateModelInertialParametersToString(std::string & model_string, 
                                                                       const iDynTree::VectorDynSize& inertialParams,
                                                                       const std::string filetype,
                                                                       const ModelExporterOptions options)
    {
        Model exportedModel = this->model();
        
        bool ok = exportedModel.updateInertialParameters(inertialParams);
        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToString", "Error in iDynTree::Model::updateInertialParameters method.");
            return false;
        }
        
        ok = m_pimpl->modelExporter.init(exportedModel, options);
        ok = ok && m_pimpl->modelExporter.exportModelToString(model_string, filetype);
        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToString", "Error in ModelExporter::exportModelToString method.");
            return false;
        }

        return true;
    }


    bool ModelCalibrationHelper::updateModelInertialParametersToFile(const std::string & filename, 
                                                                     const iDynTree::VectorDynSize& inertialParams,
                                                                     const std::string filetype,
                                                                     const ModelExporterOptions options)
    {
        Model exportedModel = this->model();

        bool ok = exportedModel.updateInertialParameters(inertialParams);
        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToFile", "Error in iDynTree::Model::updateInertialParameters method.");
            return false;
        }

        ok = m_pimpl->modelExporter.init(exportedModel, options);
        ok = ok && m_pimpl->modelExporter.exportModelToFile(filename, filetype);
        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToFile", "Error in ModelExporter::exportModelToFile method.");
            return false;
        }
        
        return true;
    }

}
