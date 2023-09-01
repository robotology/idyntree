// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_FORCETORQUESENSORELEMENT_H
#define IDYNTREE_MODELIO_URDF_FORCETORQUESENSORELEMENT_H

#include <iDynTree/XMLElement.h>
#include "SensorElement.h"

namespace iDynTree {
    class ForceTorqueSensorElement;
    class ForceTorqueSensorHelper;

    class XMLAttribute;
    class XMLParserState;
}

class iDynTree::ForceTorqueSensorElement: public iDynTree::XMLElement {
private:
    std::shared_ptr<iDynTree::ForceTorqueSensorHelper> m_helper;
public:
    explicit ForceTorqueSensorElement(
        XMLParserState& parserState, 
        std::shared_ptr<const SensorElement::SensorInfo> sensorInfo);

    std::shared_ptr<XMLElement> childElementForName(const std::string& name) override;

    const std::shared_ptr<iDynTree::SensorHelper> helper() const;
};

class iDynTree::ForceTorqueSensorHelper final: public iDynTree::SensorHelper {
    friend class ForceTorqueSensorElement;

    std::string m_measuredFrame;
    std::string m_measureDirection;
public:
    ForceTorqueSensorHelper(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo);
    Sensor* generateSensor(const Model& model) const override;
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_FORCETORQUESENSORELEMENT_H */
