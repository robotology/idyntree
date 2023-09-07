// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "ForceTorqueSensorElement.h"

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>

namespace iDynTree {

    ForceTorqueSensorHelper::ForceTorqueSensorHelper(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo)
    : SensorHelper(sensorInfo) {}

    Sensor* ForceTorqueSensorHelper::generateSensor(const Model& model) const
    {
        // Assume the model is fully formed except for the sensors
        std::string attachedJointName = m_sensorInfo->m_attachedJoint;
        if (attachedJointName.empty()) {
            std::string message = std::string("Joint name not specified for the FT sensor ") + m_sensorInfo->m_name;
            reportError("ForceTorqueSensorElement::ForceTorqueSensorInfo", "generateSensor", message.c_str());
            return nullptr;
        }
        iDynTree::JointIndex jointIndex = model.getJointIndex(attachedJointName);
        if (jointIndex == JOINT_INVALID_INDEX) {
            // TODO: error
            return nullptr;
        }

        iDynTree::Traversal traversal;
        model.computeFullTreeTraversal(traversal);

        iDynTree::LinkIndex parentLinkIndex = traversal.getParentLinkIndexFromJointIndex(model, jointIndex);
        iDynTree::LinkIndex childLinkIndex = traversal.getChildLinkIndexFromJointIndex(model, jointIndex);

        if (parentLinkIndex == LINK_INVALID_INDEX ||
            childLinkIndex == LINK_INVALID_INDEX) {
            // TODO: error
            return nullptr;
        }
        SixAxisForceTorqueSensor *sensor = new SixAxisForceTorqueSensor();

        sensor->setName(m_sensorInfo->m_name);
        sensor->setParentJoint(attachedJointName);
        sensor->setParentJointIndex(jointIndex);

        if (m_measureDirection == "parent_to_child")
        {
            sensor->setAppliedWrenchLink(childLinkIndex);
        }
        else if(m_measureDirection == "child_to_parent")
        {
            sensor->setAppliedWrenchLink(parentLinkIndex);
        }
        else
        {
            delete sensor;
            // TODO: error
//            std::cerr<< "[ERROR] sensor " << sensorName
//            << " of type force_torque has unexpected measure_direction content " << measure_direction
//            << ", parsing failed." << std::endl;
//            parsingSuccessful = false;
            return nullptr;
        }

        // The transform tag is parsed using the Gazebo convention
        // For now we assume that the six axis ft sensor is attached to a
        // fixed junction. Hence the first/second link to sensor transforms
        // are fixed are given by the frame option
        iDynTree::Transform parent_link_H_child_link = model.getJoint(jointIndex)->getRestTransform(parentLinkIndex, childLinkIndex);
        iDynTree::Transform child_link_H_sensor = m_sensorInfo->m_origin;

        if (m_measuredFrame == "child") {
            sensor->setFirstLinkSensorTransform(parentLinkIndex,parent_link_H_child_link);
            sensor->setSecondLinkSensorTransform(childLinkIndex,iDynTree::Transform::Identity());

        } else if (m_measuredFrame == "parent") {
            sensor->setFirstLinkSensorTransform(parentLinkIndex,iDynTree::Transform::Identity());
            sensor->setSecondLinkSensorTransform(childLinkIndex,parent_link_H_child_link.inverse());

        } else if (m_measuredFrame == "sensor") {
            sensor->setFirstLinkSensorTransform(parentLinkIndex,parent_link_H_child_link*child_link_H_sensor);
            sensor->setSecondLinkSensorTransform(childLinkIndex,child_link_H_sensor);
        } else {
            // Error check should have been done in the parsing. Just return NULL
            reportError("ForceTorqueSensorElement::ForceTorqueSensorInfo", "generateSensor", "Unexpected sensor frame.");
            delete sensor;
            return nullptr;
        }

        sensor->setFirstLinkName(model.getLinkName(parentLinkIndex));
        sensor->setSecondLinkName(model.getLinkName(childLinkIndex));

        return sensor;
    }

    ForceTorqueSensorElement::ForceTorqueSensorElement(
        XMLParserState& parserState,
        std::shared_ptr<const SensorElement::SensorInfo> sensorInfo)
    : iDynTree::XMLElement(parserState, "force_torque")
    , m_helper(std::make_shared<ForceTorqueSensorHelper>(sensorInfo)) {}

    const std::shared_ptr<iDynTree::SensorHelper> ForceTorqueSensorElement::helper() const
    {
        return m_helper;
    }

    std::shared_ptr<XMLElement> ForceTorqueSensorElement::childElementForName(const std::string& name)
    {
        std::shared_ptr<XMLElement> element = std::make_shared<XMLElement>(
            getParserState(), name);
        if (name == "frame") {
            std::weak_ptr<XMLElement> weakElement(element);
            element->setExitScopeCallback([this, weakElement]{
                //Get the text content
                std::shared_ptr<XMLElement> currentElement = weakElement.lock();
                std::string value = currentElement->getParsedTextContent();
                
                m_helper->m_measuredFrame = value;
            });
        }
        else if (name == "measure_direction") {
            std::weak_ptr<XMLElement> weakElement(element);
            element->setExitScopeCallback([this, weakElement]{
                //Get the text content
                std::shared_ptr<XMLElement> currentElement = weakElement.lock();
                std::string value = currentElement->getParsedTextContent();
                m_helper->m_measureDirection = value;
            });
        }
        return element;
    }

}
