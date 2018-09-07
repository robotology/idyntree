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

#include "InertialElement.h"

#include "OriginElement.h"

#include <iDynTree/XMLAttribute.h>

#include "URDFParsingUtils.h"

namespace iDynTree {
    
    InertialElement::InertialElement(iDynTree::Link &link)
    : iDynTree::XMLElement("inertial")
    , m_centerOfMass(iDynTree::Transform::Identity())
    , m_mass(0)
    , m_link(link) {}
    
    std::shared_ptr<iDynTree::XMLElement> InertialElement::childElementForName(const std::string& name) {
        // As an alternative for simple elements, instead of creating other classes,
        // I implement the simple functions of the generic element
        if (name == "origin") {
            return std::make_shared<OriginElement>(m_centerOfMass);
        }
        if (name == "mass") {
            std::shared_ptr<iDynTree::XMLElement> element = std::make_shared<iDynTree::XMLElement>(name);
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
            std::shared_ptr<iDynTree::XMLElement> element = std::make_shared<iDynTree::XMLElement>(name);
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
                m_rotationalInertiaWRTCoM = RotationalInertiaRaw(buffer, 3, 3);
                
                return true;
            });
            return element;
        }
        return std::make_shared<iDynTree::XMLElement>(name);
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
