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

#include "URDFDocument.h"
#include "LinkElement.h"
#include "InertialElement.h"

#include <iDynTree/XMLParser.h>
#include <iDynTree/Sensors/ModelSensorsTransformers.h>


#include <string>
#include <vector>

namespace iDynTree
{
    class ModelCalibrationHelper::ModelCalibrationHelperPimpl {
    public:
        Model m_model;
        SensorsList m_sensors;
        bool m_isModelValid;
        std::shared_ptr<XMLParser> m_parser;

        bool setModelAndSensors(const Model& _model, const SensorsList& _sensors);

        ModelCalibrationHelperPimpl(): m_parser(new XMLParser()) {}
    };

    bool ModelCalibrationHelper::ModelCalibrationHelperPimpl::setModelAndSensors(const Model& _model,
                                                           const SensorsList& _sensors)
    {

        m_model = _model;
        m_sensors = _sensors;

        // TODO \todo add a self consistency check of model/sensors
        m_isModelValid = true;

        if (!m_isModelValid)
        {
            reportError("ModelCalibrationHelper","setModelAndSensors","Loading failed, resetting ModelCalibrationHelper to be invalid");
            m_model = Model();
            m_sensors = SensorsList();
        }

        return m_isModelValid;
    }

    ModelCalibrationHelper::ModelCalibrationHelper()
    : m_pimpl(new ModelCalibrationHelperPimpl())
    {
        m_pimpl->m_isModelValid = false;
    }

    ModelCalibrationHelper::~ModelCalibrationHelper() {}

    const Model& ModelCalibrationHelper::model()
    {
        return m_pimpl->m_model;
    }

    const SensorsList& ModelCalibrationHelper::sensors()
    {
        return m_pimpl->m_sensors;
    }

    bool ModelCalibrationHelper::isValid()
    {
        return m_pimpl->m_isModelValid;
    }

    bool ModelCalibrationHelper::loadModelFromFile(const std::string& filename,
                                        const std::string& /*filetype*/)
    {
        // Allocate parser
        m_pimpl->m_parser = std::make_shared<XMLParser>();

        // We need to save the model model in memory
        m_pimpl->m_parser->setKeepTreeInMemory(true);

        m_pimpl->m_parser->setDocumentFactory([]{ return std::shared_ptr<XMLDocument>(new URDFDocument); });
        if (!m_pimpl->m_parser->parseXMLFile(filename)) {
            reportError("ModelCalibrationHelper", "loadModelFromFile", "Error in parsing model from URDF.");
            return false;
        }
        // Retrieving the parsed document, which is an instance of URDFDocument
        std::shared_ptr<const XMLDocument> document = m_pimpl->m_parser->document();
        std::shared_ptr<const URDFDocument> urdfDocument = std::dynamic_pointer_cast<const URDFDocument>(document);
        if (!urdfDocument) {
            reportError("ModelCalibrationHelper", "loadModelFromFile", "Fatal error in retrieving the parsed model.");
            return false;
        }

        return m_pimpl->setModelAndSensors(urdfDocument->model(),urdfDocument->sensors());
    }

    bool ModelCalibrationHelper::updateParamsHelper(const iDynTree::VectorDynSize& inertialParams) {
        if (!m_pimpl->m_isModelValid) {
            return false;
        }

        // Update inertias of the model
        bool ok = m_pimpl->m_model.updateInertialParameters(inertialParams);

        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToFile", "Error in the model updateInertialParameters method.");
            return false;
        }

        // Propagate the inertia update to the parser document
        std::shared_ptr<const XMLDocument> document = m_pimpl->m_parser->document();
        std::shared_ptr<const URDFDocument> urdfDocument = std::dynamic_pointer_cast<const URDFDocument>(document);

        // Navigate the URDFDocument and search for link elements
        auto root = urdfDocument->root();

        // Find all links elements child of root
        auto rootChildren = root->children();

        for (auto elem : rootChildren) {
            if (elem->name() == "link") {
                std::shared_ptr<LinkElement> linkElem = std::dynamic_pointer_cast<LinkElement>(elem);
                std::string linkName = linkElem->linkName();
                auto linkChildren = linkElem->children();
                for (auto childLink : linkChildren) {
                    if (childLink->name() == "inertial") {
                        std::shared_ptr<InertialElement> inertialElem = std::dynamic_pointer_cast<InertialElement>(childLink);

                        // Update the inertialElem
                        if (m_pimpl->m_model.isLinkNameUsed(linkName))
                        {
                            LinkIndex lnkIdx =  m_pimpl->m_model.getLinkIndex(linkName);
                            SpatialInertia linkInertia = m_pimpl->m_model.getLink(lnkIdx)->getInertia();

                            inertialElem->setInertia(linkInertia);
                        }
                    }
                }
            }
        }

        return true;
    }

    bool ModelCalibrationHelper::updateModelInertialParametersToString(std::string & modelString, const iDynTree::VectorDynSize& inertialParams) {

        bool ok = updateParamsHelper(inertialParams);
        if (!ok) {
            return false;
        }

        // Write the document to string
        ok =  m_pimpl->m_parser->saveToXMLString(modelString);

        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToString", "Error in saveToXMLString method.");
            return false;
        }

        return true;
    }


    bool ModelCalibrationHelper::updateModelInertialParametersToFile(const std::string & filename, const iDynTree::VectorDynSize& inertialParams)
    {
        bool ok = updateParamsHelper(inertialParams);
        if (!ok) {
            return false;
        }

        // Write the document to file
        ok =  m_pimpl->m_parser->saveToXMLFile(filename);

        if (!ok) {
            reportError("ModelCalibrationHelper", "updateModelInertialParametersToFile", "Error in saveToXMLFile method.");
            return false;
        }

        return true;
    }

}
