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

#ifndef IDYNTREE_MODELIO_URDF_URDFDOCUMENT_H
#define IDYNTREE_MODELIO_URDF_URDFDOCUMENT_H

#include <iDynTree/XMLDocument.h>

#include "JointElement.h"
#include "MaterialElement.h"
#include "SensorElement.h"
#include "VisualElement.h"
#include "iDynTree/ModelIO/ModelLoader.h"

#include <iDynTree/Model/Model.h>
#include <iDynTree/Sensors/Sensors.h>

#include <string>
#include <vector>
#include <unordered_map>

namespace iDynTree {
    class URDFDocument;
}


class iDynTree::URDFDocument: public iDynTree::XMLDocument {
    // This is the final output of the parsing + processing
    iDynTree::Model m_model;
    iDynTree::SensorsList m_sensors;

    iDynTree::ModelParserOptions m_options;

    struct {
        // Intermediate variable needed for saving the parsing result before the processing
        std::vector<std::shared_ptr<SensorHelper>> sensorHelpers;
        std::unordered_map<std::string, JointElement::JointInfo> joints;
        std::unordered_map<std::string, JointElement::JointInfo> fixedJoints;
        std::unordered_map<std::string, MaterialElement::MaterialInfo> materials;
        std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> visuals;
        std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> collisions;
        
    } m_buffers;

public:
    URDFDocument();
    virtual ~URDFDocument();

    iDynTree::ModelParserOptions& options();
    
    const iDynTree::Model& model() const;
    const iDynTree::SensorsList& sensors() const;
    
    std::shared_ptr<XMLElement> rootElementForName(const std::string& name) override;
    bool documentHasBeenParsed() override;
};


#endif /* end of include guard: IDYNTREE_MODELIO_URDF_URDFDOCUMENT_H */
