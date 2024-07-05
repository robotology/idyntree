// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "iDynTree/ModelLoader.h"

#include "URDFDocument.h"

#include <iDynTree/XMLParser.h>
#include <iDynTree/ModelTransformers.h>

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
        bool m_isModelValid;
        ModelParserOptions m_options;

        bool setModel(const Model& _model);
    };

    bool ModelLoader::ModelLoaderPimpl::setModel(const Model& _model)
    {
        m_model = _model;

        // TODO \todo add a self consistency check of model/sensors
        m_isModelValid = true;

        if (!m_isModelValid)
        {
            reportError("ModelLoader","setModel","Loading failed, resetting ModelLoader to be invalid");
            m_model = Model();
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
        return m_pimpl->m_model.sensors();
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
                                        const std::string& /*filetype*/,
                                        const std::vector<std::string>& packageDirs /* = {} */)
    {
        // Allocate parser
        std::shared_ptr<XMLParser> parser = std::make_shared<XMLParser>();
        auto parserOptions =  this->m_pimpl->m_options;
        auto documentFactoryWithOptions = [parserOptions](XMLParserState& state){
            return std::shared_ptr<XMLDocument>(new URDFDocument(state, parserOptions));
            };
        parser->setDocumentFactory(documentFactoryWithOptions);
        parser->setPackageDirs(packageDirs);
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

        return m_pimpl->setModel(urdfDocument->model());
    }

    bool ModelLoader::loadModelFromString(const std::string& modelString,
                                          const std::string& /*filetype*/,
                                          const std::vector<std::string>& packageDirs /* = {} */)
    {
        // Allocate parser
        std::shared_ptr<XMLParser> parser = std::make_shared<XMLParser>();
        auto parserOptions =  this->m_pimpl->m_options;
        auto documentFactoryWithOptions = [parserOptions](XMLParserState& state){
            return std::shared_ptr<XMLDocument>(new URDFDocument(state, parserOptions));
            };

        parser->setDocumentFactory(documentFactoryWithOptions);
        parser->setPackageDirs(packageDirs);
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

        return m_pimpl->setModel(urdfDocument->model());
    }

    bool ModelLoader::loadReducedModelFromFullModel(const Model& fullModel,
                                                    const std::vector< std::string >& consideredJoints,
                                                    const std::string filetype)
    {
        std::unordered_map<std::string, double> emptyRemovedJointPositions;
        return this->loadReducedModelFromFullModel(fullModel, consideredJoints, emptyRemovedJointPositions, filetype);
    }

    bool ModelLoader::loadReducedModelFromString(const std::string modelString,
                                                 const std::vector< std::string >& consideredJoints,
                                                 const std::string filetype,
                                                 const std::vector<std::string>& packageDirs /*= {}*/)
    {
        std::unordered_map<std::string, double> emptyRemovedJointPositions;
        return this->loadReducedModelFromString(modelString, consideredJoints, emptyRemovedJointPositions, filetype, packageDirs);
    }

    bool ModelLoader::loadReducedModelFromFile(const std::string filename,
                                               const std::vector< std::string >& consideredJoints,
                                               const std::string filetype,
                                               const std::vector<std::string>& packageDirs /*= {}*/)
    {
        std::unordered_map<std::string, double> emptyRemovedJointPositions;
        return this->loadReducedModelFromFile(filename, consideredJoints, emptyRemovedJointPositions, filetype, packageDirs);
    }

    bool ModelLoader::loadReducedModelFromFullModel(const Model& fullModel,
                                                    const std::vector< std::string >& consideredJoints,
                                                    const std::unordered_map<std::string, double>& removedJointPositions,
                                                    const std::string /*filetype*/)
    {
        Model _modelReduced;
        _modelReduced.setPackageDirs(fullModel.getPackageDirs());
        bool ok = createReducedModel(fullModel,consideredJoints,_modelReduced, removedJointPositions);

        if( !ok )
        {
            return false;
        }

        return m_pimpl->setModel(_modelReduced);
    }

    bool ModelLoader::loadReducedModelFromString(const std::string modelString,
                                                 const std::vector< std::string >& consideredJoints,
                                                 const std::unordered_map<std::string, double>& removedJointPositions,
                                                 const std::string filetype,
                                                 const std::vector<std::string>& packageDirs /*= {}*/)
    {
        bool parsingCorrect = loadModelFromString(modelString, filetype, packageDirs);
        if (!parsingCorrect) return false;
        Model _modelFull = m_pimpl->m_model, _modelReduced;
        _modelReduced.setPackageDirs(packageDirs);

        parsingCorrect = createReducedModel(_modelFull, consideredJoints,
                                            _modelReduced, removedJointPositions);

        if (!parsingCorrect)
        {
            return false;
        }

        return m_pimpl->setModel(_modelReduced);
    }

    bool ModelLoader::loadReducedModelFromFile(const std::string filename,
                                               const std::vector< std::string >& consideredJoints,
                                               const std::unordered_map<std::string, double>& removedJointPositions,
                                               const std::string filetype,
                                               const std::vector<std::string>& packageDirs /*= {}*/)
    {
        bool parsingCorrect = loadModelFromFile(filename, filetype, packageDirs);
        if (!parsingCorrect) return false;
        Model _modelFull = m_pimpl->m_model, _modelReduced;
        _modelReduced.setPackageDirs(packageDirs);

        parsingCorrect = createReducedModel(_modelFull,consideredJoints,_modelReduced,removedJointPositions);

        if (!parsingCorrect)
        {
            return false;
        }

        return m_pimpl->setModel(_modelReduced);
    }
}
