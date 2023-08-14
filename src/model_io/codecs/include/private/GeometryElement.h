// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_GEOMETRYELEMENT_H
#define IDYNTREE_MODELIO_URDF_GEOMETRYELEMENT_H

#include <iDynTree/XMLElement.h>
#include <vector>

namespace iDynTree {
    class GeometryElement;
    
    class SolidShape;
    class XMLParserState;
}

class iDynTree::GeometryElement: public iDynTree::XMLElement {
private:
    std::shared_ptr<SolidShape>& m_shape;
    const std::vector<std::string>& packageDirs;
public:
    explicit GeometryElement(
        XMLParserState& parserState, 
        std::shared_ptr<SolidShape>& shape, 
        const std::vector<std::string>& packageDirs);

    std::shared_ptr<iDynTree::XMLElement> childElementForName(const std::string& name) override;
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_GEOMETRYELEMENT_H */

