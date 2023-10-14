// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_INERTIAELEMENT_H
#define IDYNTREE_MODELIO_URDF_INERTIAELEMENT_H

#include <iDynTree/XMLElement.h>

#include <iDynTree/Transform.h>
#include <iDynTree/Model.h>

namespace iDynTree {
    class InertialElement;
    class XMLParserState;
}

class iDynTree::InertialElement: public iDynTree::XMLElement {
    Transform m_centerOfMass;
    double m_mass;
    RotationalInertia m_rotationalInertiaWRTCoM;
    iDynTree::Link &m_link;
    
public:
    explicit InertialElement(XMLParserState& parserState, iDynTree::Link &link);
    
    std::shared_ptr<iDynTree::XMLElement> childElementForName(const std::string& name) override;
    
    virtual void exitElementScope() override;
    
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_INERTIAELEMENT_H */
