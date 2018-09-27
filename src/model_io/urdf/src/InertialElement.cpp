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

    void InertialElement::setInertia(iDynTree::SpatialInertia& inertia) {
        m_mass = inertia.getMass();
        m_centerOfMass = iDynTree::Transform(Rotation::Identity(), inertia.getCenterOfMass());
        m_rotationalInertiaWRTCoM = inertia.getRotationalInertiaWrtCenterOfMass();
        m_link.setInertia(inertia);
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

    std::string InertialElement::description(const size_t depth) const {
        std::string indent = "  ";
        std::string totalIndent = "";
        for (size_t i=0; i < depth; i++) {
            totalIndent = totalIndent + indent;
        }

        std::string totalIndentPlusOne = totalIndent + indent;

        std::ostringstream str;
        str << totalIndent << "<inertial>" << std::endl;

        // TODO(traversaro) : handle locale
        Position com_wrt_link = m_centerOfMass.getPosition();
        Rotation link_R_com   = m_centerOfMass.getRotation();
        RotationalInertiaRaw rotInertia = link_R_com*m_rotationalInertiaWRTCoM;


        // Mass
        str << totalIndentPlusOne << "<mass value=\"" << m_mass << "\" >" << std::endl;
        str << totalIndentPlusOne << "</mass>" << std::endl;
        str << totalIndentPlusOne << "<origin rpy=\"0.0 0.0 0.0\" xyz=\"" << com_wrt_link(0)
                                                                          << " "
                                                                          << com_wrt_link(1)
                                                                          << " "
                                                                          << com_wrt_link(2)
                                                                          << "\">" << std::endl;
        str << totalIndentPlusOne << "</origin>" << std::endl;
        str << totalIndentPlusOne << "<inertia ixx=\"" << m_rotationalInertiaWRTCoM(0, 0) << "\" "
                                           << "ixy=\"" << m_rotationalInertiaWRTCoM(0, 1) << "\" "
                                           << "ixz=\"" << m_rotationalInertiaWRTCoM(0, 2) << "\" "
                                           << "iyy=\"" << m_rotationalInertiaWRTCoM(1, 1) << "\" "
                                           << "iyz=\"" << m_rotationalInertiaWRTCoM(1, 2) << "\" "
                                           << "izz=\"" << m_rotationalInertiaWRTCoM(2, 2) << "\" "
                                                       << ">" << std::endl;
        str << totalIndentPlusOne << "</inertia>" << std::endl;

        str << totalIndent << "</inertial>" << std::endl;
        return str.str();
    }

    
}
