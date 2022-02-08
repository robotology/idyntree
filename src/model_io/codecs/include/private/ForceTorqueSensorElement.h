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

#ifndef IDYNTREE_MODELIO_URDF_FORCETORQUESENSORELEMENT_H
#define IDYNTREE_MODELIO_URDF_FORCETORQUESENSORELEMENT_H

#include <iDynTree/XMLElement.h>
#include "SensorElement.h"

namespace iDynTree {
    class ForceTorqueSensorElement;
    class ForceTorqueSensorHelper;

    class XMLAttribute;
}

class iDynTree::ForceTorqueSensorElement: public iDynTree::XMLElement {
private:
    std::shared_ptr<iDynTree::ForceTorqueSensorHelper> m_helper;
public:
    ForceTorqueSensorElement(std::shared_ptr<const SensorElement::SensorInfo> sensorInfo);

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
