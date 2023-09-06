// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_SENSORELEMENT_H
#define IDYNTREE_MODELIO_URDF_SENSORELEMENT_H

#include <iDynTree/XMLElement.h>

#include <iDynTree/Transform.h>
#include <iDynTree/Sensors.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace iDynTree {
    class SensorElement;
    class SensorHelper;
    class AccelerometerSensorHelper;
    class GyroscopeSensorHelper;

    class XMLAttribute;
    class Sensor;
    class Model;
    class XMLParserState;
}


class iDynTree::SensorElement: public iDynTree::XMLElement {
public:
    // Class to keep the data needed to generate the actual Sensor implementation.
    // The only purpose is to keep it separate from the XML Element class
    struct SensorInfo {
    public:
        std::string m_name;
        // Explicitly keeping separate link and joint for error checking
        std::string m_attachedLink;
        std::string m_attachedJoint;
        iDynTree::SensorType m_sensorType;

        iDynTree::Transform m_origin;
    };

private:
    std::shared_ptr<SensorElement::SensorInfo> m_info;
    std::vector<std::shared_ptr<SensorHelper>>& m_sensors;
public:

    // I didn't find another clean way to return the generated Helper, instead of manually adding to
    // the vector
    explicit SensorElement(
        XMLParserState& parserState, 
        std::vector<std::shared_ptr<SensorHelper>>& sensors);

    std::shared_ptr<XMLElement> childElementForName(const std::string& name) override;
    bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) override;
};


class iDynTree::SensorHelper {
protected:
    std::shared_ptr<const SensorElement::SensorInfo> m_sensorInfo;
public:
    SensorHelper(std::shared_ptr<const SensorElement::SensorInfo> m_sensorInfo);

    virtual ~SensorHelper();
    // Plain old pointer as this should be used in the other iDynTree codebase
    // the called is the owner of the returned pointer, i.e. you have to manage the memory
    virtual Sensor* generateSensor(const Model&) const = 0;
};

// Helpers for sensors that do not have a tag associated

class iDynTree::AccelerometerSensorHelper final: public iDynTree::SensorHelper {
public:
    AccelerometerSensorHelper(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo);
    Sensor* generateSensor(const Model& model) const override;
};

class iDynTree::GyroscopeSensorHelper final: public iDynTree::SensorHelper {
public:
    GyroscopeSensorHelper(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo);
    Sensor* generateSensor(const Model& model) const override;
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_SENSORELEMENT_H */
