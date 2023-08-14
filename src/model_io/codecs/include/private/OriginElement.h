// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_ORIGINELEMENT_H
#define IDYNTREE_MODELIO_URDF_ORIGINELEMENT_H

#include <iDynTree/XMLElement.h>

#include <unordered_map>

namespace iDynTree {
    class OriginElement;

    class XMLAttribute;
    class Transform;
    class XMLParserState;
}

class iDynTree::OriginElement: public iDynTree::XMLElement {
private:
    iDynTree::Transform& m_jointOrigin;

public:
    explicit OriginElement(XMLParserState& parserState, iDynTree::Transform& jointOrigin);
    
    bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) override;

};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_ORIGINELEMENT_H */
