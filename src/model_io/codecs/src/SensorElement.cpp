/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Author: Francesco Romano - Google LLC
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "SensorElement.h"

#include "OriginElement.h"
#include "ForceTorqueSensorElement.h"

#include <iDynTree/XMLAttribute.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/GyroscopeSensor.h>


namespace iDynTree {

    SensorHelper::SensorHelper(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo)
    : m_sensorInfo(sensorInfo) {}
    SensorHelper::~SensorHelper() {}


    SensorElement::SensorElement(std::vector<std::shared_ptr<SensorHelper>>& sensors)
    : iDynTree::XMLElement("sensor")
    , m_info(std::make_shared<SensorInfo>())
    , m_sensors(sensors)
    {
        // Defaulting the origin
        m_info->m_origin = Transform::Identity();
    }

    std::shared_ptr<XMLElement> SensorElement::childElementForName(const std::string& name)
    {
        if (name == "origin") {
            return std::make_shared<OriginElement>(m_info->m_origin);
        } else if (name == "parent") {
            std::shared_ptr<XMLElement> element = std::make_shared<XMLElement>(name);
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) {
                auto found = attributes.find("link");
                if (found != attributes.end()) {
                    m_info->m_attachedLink = found->second->value();
                }
                found = attributes.find("joint");
                if (found != attributes.end()) {
                    m_info->m_attachedJoint = found->second->value();
                }
                return true;
            });
            return element;
        } else if (name == "force_torque") {
            if (m_info->m_sensorType != SIX_AXIS_FORCE_TORQUE) {
                // Error
            }

            std::shared_ptr<ForceTorqueSensorElement> element = std::make_shared<ForceTorqueSensorElement>(m_info);
            // We have to save the helper which is responsible of generating the iDynTree::Sensor class
            // after the XML parsing.
            m_sensors.push_back(element->helper());
            return element;
        }
        return std::make_shared<XMLElement>(name);
    }

    bool SensorElement::setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes)
    {
        auto found = attributes.find("name");
        if (found == attributes.end()) {
            reportError("SensorElement", "setAttributes", "name is required for the <sensor> element.");
            return false;
        }
        m_info->m_name = found->second->value();

        found = attributes.find("type");
        if (found == attributes.end()) {
            reportError("SensorElement", "setAttributes", "type is required for the <sensor> element.");
            return false;
        }
        std::string type = found->second->value();

        // For the sensors that do not have a tag associated, we create an helper directly
        if (type == "accelerometer") {
            m_info->m_sensorType = ACCELEROMETER;
            m_sensors.push_back(std::make_shared<AccelerometerSensorHelper>(m_info));
        } else if (type == "gyroscope") {
            m_info->m_sensorType = GYROSCOPE;
            m_sensors.push_back(std::make_shared<GyroscopeSensorHelper>(m_info));
        } else if (type == "force_torque") {
            m_info->m_sensorType = SIX_AXIS_FORCE_TORQUE;
        } else {
            std::string message = "iDynTree does not support sensor of type " + type;
            reportWarning("SensorElement", "setAttributes", message.c_str());
        }

        return true;
    }


    AccelerometerSensorHelper::AccelerometerSensorHelper(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo)
    : iDynTree::SensorHelper(sensorInfo) {}

    Sensor* AccelerometerSensorHelper::generateSensor(const Model& model) const
    {
        if (m_sensorInfo->m_attachedLink.empty()) {
            //TODO: error
            return nullptr;
        }
        iDynTree::LinkIndex linkIndex = model.getLinkIndex(m_sensorInfo->m_attachedLink);

        AccelerometerSensor * sensor = new AccelerometerSensor();
        sensor->setLinkSensorTransform(m_sensorInfo->m_origin);
        sensor->setName(m_sensorInfo->m_name);
        sensor->setParentLink(m_sensorInfo->m_attachedLink);
        sensor->setParentLinkIndex(linkIndex);
        return sensor;
    }

    GyroscopeSensorHelper::GyroscopeSensorHelper(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo)
    : iDynTree::SensorHelper(sensorInfo) {}

    Sensor*  GyroscopeSensorHelper::generateSensor(const Model& model) const
    {
        if (m_sensorInfo->m_attachedLink.empty()) {
            //TODO: error
            return nullptr;
        }
        iDynTree::LinkIndex linkIndex = model.getLinkIndex(m_sensorInfo->m_attachedLink);

        GyroscopeSensor * sensor = new GyroscopeSensor();
        sensor->setLinkSensorTransform(m_sensorInfo->m_origin);
        sensor->setName(m_sensorInfo->m_name);
        sensor->setParentLink(m_sensorInfo->m_attachedLink);
        sensor->setParentLinkIndex(linkIndex);
        return sensor;
    }
    
}
