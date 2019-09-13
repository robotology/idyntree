/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/ModelIO/ModelCalibrationHelper.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/ModelIO/ModelExporter.h>

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
        SensorsList exportedSensors = this->sensors();
        
        bool ok = exportedModel.updateInertialParameters(inertialParams);
        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToString", "Error in iDynTree::Model::updateInertialParameters method.");
            return false;
        }
        
        ok = m_pimpl->modelExporter.init(exportedModel, exportedSensors, options);
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
        SensorsList exportedSensors = this->sensors();

        bool ok = exportedModel.updateInertialParameters(inertialParams);
        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToFile", "Error in iDynTree::Model::updateInertialParameters method.");
            return false;
        }

        ok = m_pimpl->modelExporter.init(exportedModel, exportedSensors, options);
        ok = ok && m_pimpl->modelExporter.exportModelToFile(filename, filetype);
        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToFile", "Error in ModelExporter::exportModelToFile method.");
            return false;
        }
        
        return true;
    }

}
