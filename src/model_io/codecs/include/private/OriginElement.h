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

#ifndef IDYNTREE_MODELIO_URDF_ORIGINELEMENT_H
#define IDYNTREE_MODELIO_URDF_ORIGINELEMENT_H

#include <iDynTree/XMLElement.h>

#include <unordered_map>

namespace iDynTree {
    class OriginElement;

    class XMLAttribute;
    class Transform;
}

class iDynTree::OriginElement: public iDynTree::XMLElement {
private:
    iDynTree::Transform& m_jointOrigin;

public:
    OriginElement(iDynTree::Transform& jointOrigin);
    
    bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) override;

};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_ORIGINELEMENT_H */
