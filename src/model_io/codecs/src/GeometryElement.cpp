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

#include "GeometryElement.h"

#include "URDFParsingUtils.h"

#include <iDynTree/XMLAttribute.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Model/SolidShapes.h>

namespace iDynTree{

    GeometryElement::GeometryElement(std::shared_ptr<SolidShape>& shape)
    : iDynTree::XMLElement("geometry")
    , m_shape(shape) {}

    std::shared_ptr<iDynTree::XMLElement> GeometryElement::childElementForName(const std::string& name)
    {
        std::shared_ptr<XMLElement> element = std::make_shared<XMLElement>(name);
        if (name == "box") {
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>> &attributes){

                auto found = attributes.find("size");
                if (found == attributes.end()) {
                    reportError("GeometryElement", "childElementForName::box", "Missing 'size' attribute for box geometry.");
                    return false;
                }
                Vector3 boxDimensionn;
                if (!vector3FromString(found->second->value(), boxDimensionn)) {

                    return false;
                }
                Box *box = new Box();

                box->setX(boxDimensionn(0));
                box->setY(boxDimensionn(1));
                box->setZ(boxDimensionn(2));

                m_shape = std::shared_ptr<SolidShape>(box);
                return true;
            });
        } else if (name == "cylinder") {
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute> > &attributes){

                double radius, length;
                auto found = attributes.find("radius");
                if (found == attributes.end()) {
                    reportError("GeometryElement", "childElementForName::cylinder", "Missing 'radius' attribute for cylinder geometry.");
                    return false;
                }
                if (!stringToDoubleWithClassicLocale(found->second->value(), radius)) {
                    reportError("GeometryElement", "childElementForName::cylinder", "Failed to parse 'radius' attribute for cylinder geometry.");
                    return false;
                }

                found = attributes.find("length");
                if (found == attributes.end()) {
                    reportError("GeometryElement", "childElementForName::cylinder", "Missing 'length' attribute for cylinder geometry.");
                    return false;
                }
                if (!stringToDoubleWithClassicLocale(found->second->value(), length)) {
                    reportError("GeometryElement", "childElementForName::cylinder", "Failed to parse 'length' attribute for cylinder geometry.");
                    return false;
                }
                Cylinder * cylinder = new Cylinder();
                cylinder->setRadius(radius);
                cylinder->setLength(length);

                m_shape = std::shared_ptr<Cylinder>(cylinder);
                return true;
            });
        } else if (name == "sphere") {
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute> > &attributes){

                double radius;
                auto found = attributes.find("radius");
                if (found == attributes.end()) {
                    reportError("GeometryElement", "childElementForName::sphere", "Missing 'radius' attribute for sphere geometry.");
                    return false;
                }
                if (!stringToDoubleWithClassicLocale(found->second->value(), radius)) {
                    reportError("GeometryElement", "childElementForName::sphere", "Failed to parse 'radius' attribute for sphere geometry.");
                    return false;
                }

                Sphere * sphere = new Sphere();
                sphere->setRadius(radius);

                m_shape = std::shared_ptr<Sphere>(sphere);
                return true;
            });
        } else if (name == "mesh") {
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute> > &attributes){

                auto found = attributes.find("filename");
                if (found == attributes.end()) {
                    reportWarning("GeometryElement", "childElementForName::mesh", "Missing 'filename' attribute for sphere mesh.");
                } else {
                    //TODO: Do not fail (?)
                    // For now we just support urdf with local meshes, see as an example
                    // https://github.com/bulletphysics/bullet3/tree/master/data
                    ExternalMesh * externalMesh = new ExternalMesh();
                    externalMesh->setFilename(found->second->value());
                    //                    pExternalMesh->filename = getURDFMeshAbsolutePathFilename(urdf_filename,localName);
                    iDynTree::Vector3 scale;
                    scale(0) = scale(1) = scale(2) = 1.0;

                    found = attributes.find("scale");
                    if (found != attributes.end()) {
                        vector3FromString(found->second->value(), scale);
                    }
                    externalMesh->setScale(scale);
                    m_shape = std::shared_ptr<ExternalMesh>(externalMesh);
                }
                return true;
            });
        }
        return element;
    }

}
