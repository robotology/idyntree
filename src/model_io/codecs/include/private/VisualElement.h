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

#ifndef IDYNTREE_MODELIO_URDF_VISUALELEMENT_H
#define IDYNTREE_MODELIO_URDF_VISUALELEMENT_H

#include <iDynTree/XMLElement.h>

#include "MaterialElement.h"

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Model/SolidShapes.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace iDynTree {
    class VisualElement;

    class XMLAttribute;
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
    };

private:
    VisualInfo m_info;

public:
    VisualElement(const std::string& name);

    const VisualInfo& visualInfo() const;

    bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) override;
    std::shared_ptr<iDynTree::XMLElement> childElementForName(const std::string& name) override;
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_VISUALELEMENT_H */

