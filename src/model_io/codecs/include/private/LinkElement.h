// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_LINKELEMENT_H
#define IDYNTREE_MODELIO_URDF_LINKELEMENT_H

#include <iDynTree/XMLElement.h>

#include "VisualElement.h"

#include <iDynTree/Link.h>

namespace iDynTree {
    class LinkElement;
    class XMLAttribute;
    
    class Model;
    class XMLParserState;
}

class iDynTree::LinkElement : public iDynTree::XMLElement {
    iDynTree::Model& m_model;

    iDynTree::Link m_link;
    std::string m_linkName;
    std::vector<VisualElement::VisualInfo> m_visuals;
    std::vector<VisualElement::VisualInfo> m_collisions;
    
public:
    explicit LinkElement(XMLParserState& parserState, iDynTree::Model &model);

    // Exposing useful properties
    const std::string& linkName() const;
    const std::vector<VisualElement::VisualInfo>& visuals() const;
    const std::vector<VisualElement::VisualInfo>& collisions() const;
    
    void exitElementScope() override;
    
    bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) override;

    void childHasBeenParsed(std::shared_ptr<iDynTree::XMLElement> child) override;
    
    std::shared_ptr<iDynTree::XMLElement> childElementForName(const std::string& name) override;
    
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_LINKELEMENT_H */
