// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_VISUALELEMENT_H
#define IDYNTREE_MODELIO_URDF_VISUALELEMENT_H

#include <iDynTree/XMLElement.h>

#include "MaterialElement.h"

#include <iDynTree/Transform.h>
#include <iDynTree/SolidShapes.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace iDynTree {
    class VisualElement;

    class XMLAttribute;
    class XMLParserState;
}

class iDynTree::VisualElement: public iDynTree::XMLElement
{
public:
    struct VisualInfo {
        std::string m_name;
        bool m_nameAttributeFound;
        iDynTree::Transform m_origin{iDynTree::Transform::Identity()};
        std::shared_ptr<SolidShape> m_solidShape;
        std::shared_ptr<MaterialElement::MaterialInfo> m_material;
        const std::vector<std::string>& m_packageDirs;

        VisualInfo(const std::vector<std::string>& packageDirs);
    };

private:
    VisualInfo m_info;

public:
    explicit VisualElement(
        XMLParserState& parserState,
        const std::string& name,
        const std::vector<std::string>& packageDirs);

    const VisualInfo& visualInfo() const;

    bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) override;
    std::shared_ptr<iDynTree::XMLElement> childElementForName(const std::string& name) override;
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_VISUALELEMENT_H */

