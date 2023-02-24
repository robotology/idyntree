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

