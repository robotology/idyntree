/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/ModelIO/ModelLoader.h"

#include "URDFDocument.h"

#include <iDynTree/XMLParser.h>
#include <iDynTree/Sensors/ModelSensorsTransformers.h>


#include <string>
#include <vector>

namespace iDynTree
{

    ModelParserOptions::ModelParserOptions()
    : addSensorFramesAsAdditionalFrames(true)
    , originalFilename("") {}


    class ModelLoader::ModelLoaderPimpl {
    public:
        Model m_model;
        SensorsList m_sensors;
        bool m_isModelValid;
        ModelParserOptions m_options;

        bool setModelAndSensors(const Model& _model, const SensorsList& _sensors);
    };

    bool ModelLoader::ModelLoaderPimpl::setModelAndSensors(const Model& _model,
                                                           const SensorsList& _sensors)
    {

        m_model = _model;
        m_sensors = _sensors;

        // TODO \todo add a self consistency check of model/sensors
        m_isModelValid = true;

        if (!m_isModelValid)
        {
            reportError("ModelLoader","setModelAndSensors","Loading failed, resetting ModelLoader to be invalid");
            m_model = Model();
            m_sensors = SensorsList();
        }

        return m_isModelValid;
    }

    ModelLoader::ModelLoader()
    : m_pimpl(new ModelLoaderPimpl())
    {
        m_pimpl->m_isModelValid = false;
    }

    ModelLoader::~ModelLoader() {}

    const Model& ModelLoader::model()
    {
        return m_pimpl->m_model;
    }

    const SensorsList& ModelLoader::sensors()
    {
        return m_pimpl->m_sensors;
    }

    bool ModelLoader::isValid()
    {
        return m_pimpl->m_isModelValid;
    }


    const ModelParserOptions& ModelLoader::parsingOptions() const { return m_pimpl->m_options; }

    void ModelLoader::setParsingOptions(const ModelParserOptions& options)
    {
        m_pimpl->m_options = options;
    }

    bool ModelLoader::loadModelFromFile(const std::string& filename,
                                        const std::string& /*filetype*/)
    {
        // Allocate parser
        std::shared_ptr<XMLParser> parser = std::make_shared<XMLParser>();
        parser->setDocumentFactory([]{ return std::shared_ptr<XMLDocument>(new URDFDocument); });
        if (!parser->parseXMLFile(filename)) {
            reportError("ModelLoader", "loadModelFromFile", "Error in parsing model from URDF.");
            return false;
        }
        // Retrieving the parsed document, which is an instance of URDFDocument
        std::shared_ptr<const XMLDocument> document = parser->document();
        std::shared_ptr<const URDFDocument> urdfDocument = std::dynamic_pointer_cast<const URDFDocument>(document);
        if (!urdfDocument) {
            reportError("ModelLoader", "loadModelFromFile", "Fatal error in retrieving the parsed model.");
            return false;
        }

        return m_pimpl->setModelAndSensors(urdfDocument->model(),urdfDocument->sensors());
    }

    bool ModelLoader::loadModelFromString(const std::string& modelString,
                                          const std::string& /*filetype*/)
    {
        // Allocate parser
        std::shared_ptr<XMLParser> parser = std::make_shared<XMLParser>();
        parser->setDocumentFactory([]{ return std::shared_ptr<XMLDocument>(new URDFDocument); });
        if (!parser->parseXMLString(modelString)) {
            reportError("ModelLoader", "loadModelFromString", "Error in parsing model from URDF.");
            return false;
        }
        // Retrieving the parsed document, which is an instance of URDFDocument
        std::shared_ptr<const XMLDocument> document = parser->document();
        std::shared_ptr<const URDFDocument> urdfDocument = std::dynamic_pointer_cast<const URDFDocument>(document);
        if (!urdfDocument) {
            reportError("ModelLoader", "loadModelFromString", "Fatal error in retrieving the parsed model.");
            return false;
        }

        return m_pimpl->setModelAndSensors(urdfDocument->model(),urdfDocument->sensors());
    }

    bool ModelLoader::loadReducedModelFromFullModel(const Model& fullModel,
                                                    const std::vector< std::string >& consideredJoints,
                                                    const std::string /*filetype*/)
    {
        SensorsList _sensorsFull, _sensorsReduced;
        Model _modelReduced;
        bool ok = createReducedModelAndSensors(fullModel,_sensorsFull,consideredJoints,_modelReduced,_sensorsReduced);

        if( !ok )
        {
            return false;
        }

        return m_pimpl->setModelAndSensors(_modelReduced,_sensorsReduced);
    }

    bool ModelLoader::loadReducedModelFromString(const std::string modelString,
                                                 const std::vector< std::string >& consideredJoints,
                                                 const std::string /*filetype*/)
    {
        bool parsingCorrect = loadModelFromString(modelString);
        if (!parsingCorrect) return false;
        SensorsList _sensorsFull = m_pimpl->m_sensors, _sensorsReduced;
        Model _modelFull = m_pimpl->m_model, _modelReduced;

        parsingCorrect = createReducedModelAndSensors(_modelFull, _sensorsFull,
                                                      consideredJoints,
                                                      _modelReduced, _sensorsReduced);

        if (!parsingCorrect)
        {
            return false;
        }

        return m_pimpl->setModelAndSensors(_modelReduced, _sensorsReduced);
    }

    bool ModelLoader::loadReducedModelFromFile(const std::string filename,
                                               const std::vector< std::string >& consideredJoints,
                                               const std::string /*filetype*/)
    {
        bool parsingCorrect = loadModelFromFile(filename);
        if (!parsingCorrect) return false;
        SensorsList _sensorsFull = m_pimpl->m_sensors, _sensorsReduced;
        Model _modelFull = m_pimpl->m_model, _modelReduced;

        parsingCorrect = createReducedModelAndSensors(_modelFull,_sensorsFull,consideredJoints,_modelReduced,_sensorsReduced);

        if (!parsingCorrect)
        {
            return false;
        }

        return m_pimpl->setModelAndSensors(_modelReduced,_sensorsReduced);
    }
}
