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

#ifndef IDYNTREE_MODELIO_URDF_INERTIAELEMENT_H
#define IDYNTREE_MODELIO_URDF_INERTIAELEMENT_H

#include <iDynTree/XMLElement.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Model/Model.h>

namespace iDynTree {
    class InertialElement;
}

class iDynTree::InertialElement: public iDynTree::XMLElement {
    Transform m_centerOfMass;
    double m_mass;
    RotationalInertiaRaw m_rotationalInertiaWRTCoM;
    iDynTree::Link &m_link;
    
public:
    InertialElement(iDynTree::Link &link);
    
    std::shared_ptr<iDynTree::XMLElement> childElementForName(const std::string& name) override;
    
    virtual void exitElementScope() override;
    
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_INERTIAELEMENT_H */
