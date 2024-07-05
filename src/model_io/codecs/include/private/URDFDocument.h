// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_URDFDOCUMENT_H
#define IDYNTREE_MODELIO_URDF_URDFDOCUMENT_H

#include <iDynTree/XMLDocument.h>

#include "JointElement.h"
#include "MaterialElement.h"
#include "SensorElement.h"
#include "VisualElement.h"
#include "iDynTree/ModelLoader.h"

#include <iDynTree/Model.h>
#include <iDynTree/Sensors.h>

#include <string>
#include <vector>
#include <unordered_map>

namespace iDynTree {
    class URDFDocument;
    class XMLParserState;
}


class iDynTree::URDFDocument: public iDynTree::XMLDocument {
    // This is the final output of the parsing + processing
    iDynTree::Model m_model;

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
    explicit URDFDocument(XMLParserState& parserState, const iDynTree::ModelParserOptions& parserOptions);
    virtual ~URDFDocument();

    iDynTree::ModelParserOptions& options();
    
    const iDynTree::Model& model() const;
    const iDynTree::SensorsList& sensors() const;
    
    std::shared_ptr<XMLElement> rootElementForName(const std::string& name,
                                                   const std::vector<std::string>& packageDirs) override;
    bool documentHasBeenParsed() override;
};


#endif /* end of include guard: IDYNTREE_MODELIO_URDF_URDFDOCUMENT_H */
