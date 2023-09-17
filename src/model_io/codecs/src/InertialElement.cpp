// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "InertialElement.h"

#include "OriginElement.h"

#include <iDynTree/XMLAttribute.h>

#include "URDFParsingUtils.h"

namespace iDynTree {
    
    InertialElement::InertialElement(
        XMLParserState& parserState, 
        iDynTree::Link &link)
    : iDynTree::XMLElement(parserState, "inertial")
    , m_centerOfMass(iDynTree::Transform::Identity())
    , m_mass(0)
    , m_link(link) {}
    
    std::shared_ptr<iDynTree::XMLElement> InertialElement::childElementForName(const std::string& name) {
        // As an alternative for simple elements, instead of creating other classes,
        // I implement the simple functions of the generic element
        if (name == "origin") {
            return std::make_shared<OriginElement>(getParserState(), m_centerOfMass);
        }
        if (name == "mass") {
            std::shared_ptr<iDynTree::XMLElement> element = std::make_shared<iDynTree::XMLElement>(
                getParserState(), name);
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) {
                m_mass = 0;
                auto mass = attributes.find("value");
                if (mass != attributes.end()
                    && !stringToDoubleWithClassicLocale(mass->second->value(), m_mass)) {
                    reportWarning("InertiaElement", "childElementForName::mass::f_attribute", "Failed to obtain floating point representation for the mass element.");
                }
                return true;
            });
            return element;
        }
        if (name == "inertia") {
            std::shared_ptr<iDynTree::XMLElement> element = std::make_shared<iDynTree::XMLElement>(
                getParserState(), name);
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) {
                auto ixx = attributes.find("ixx");
                auto ixy = attributes.find("ixy");
                auto ixz = attributes.find("ixz");
                auto iyy = attributes.find("iyy");
                auto izz = attributes.find("izz");
                auto iyz = attributes.find("iyz");
                
                if (ixx == attributes.end()
                    || ixy == attributes.end()
                    || ixz == attributes.end()
                    || iyy == attributes.end()
                    || izz == attributes.end()
                    || iyz == attributes.end()) {
                    reportError("InertialElement", "childElementForName::inertia::f_attribute", "Missing some inertia parameter. Expecting 6");
                    return false;
                }
                double xx;
                double xy;
                double xz;
                double yy;
                double zz;
                double yz;
                if (!stringToDoubleWithClassicLocale(ixx->second->value(), xx)
                    || !stringToDoubleWithClassicLocale(ixy->second->value(), xy)
                    || !stringToDoubleWithClassicLocale(ixz->second->value(), xz)
                    || !stringToDoubleWithClassicLocale(iyy->second->value(), yy)
                    || !stringToDoubleWithClassicLocale(izz->second->value(), zz)
                    || !stringToDoubleWithClassicLocale(iyz->second->value(), yz)) {
                    reportWarning("InertiaElement", "childElementForName::inertia::f_attribute", "Failed to obtain floating point representation for the inertia element.");
                    return false;
                }
                double buffer[3 * 3] = {
                    xx, xy, xz,
                    xy, yy, yz,
                    xz, yz, zz
                };
                m_rotationalInertiaWRTCoM = RotationalInertia(buffer, 3, 3);
                
                return true;
            });
            return element;
        }
        return std::make_shared<iDynTree::XMLElement>(getParserState(), name);
    }
    
    void InertialElement::exitElementScope() {
        Position com_wrt_link = m_centerOfMass.getPosition();
        Rotation link_R_com   = m_centerOfMass.getRotation();
        SpatialInertia inertia;
        inertia.fromRotationalInertiaWrtCenterOfMass(m_mass,
                                                     com_wrt_link,
                                                     link_R_com * m_rotationalInertiaWRTCoM);

        m_link.setInertia(inertia);
    }
    
}
