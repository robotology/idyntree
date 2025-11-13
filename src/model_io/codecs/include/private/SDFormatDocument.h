// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_SDFORMAT_SDFORMATDOCUMENT_H
#define IDYNTREE_MODELIO_SDFORMAT_SDFORMATDOCUMENT_H

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Sensors.h>

#include <memory>
#include <string>
#include <vector>

namespace iDynTree
{
class SDFormatDocument;
} // namespace iDynTree

class iDynTree::SDFormatDocument
{
    // This is the final output of the parsing + processing
    iDynTree::Model m_model;

    iDynTree::ModelParserOptions m_options;

#ifdef IDYNTREE_USES_SDFORMAT
    // Store the parsed SDF model for conversion
    std::shared_ptr<const void> m_sdfModel; // Using void* to avoid exposing sdf types in header
#endif

public:
    explicit SDFormatDocument(const iDynTree::ModelParserOptions& parserOptions);
    virtual ~SDFormatDocument();

    iDynTree::ModelParserOptions& options();

    const iDynTree::Model& model() const;
    const iDynTree::SensorsList& sensors() const;

    // Helper method to load from SDF file using sdformat library
    bool loadFromFile(const std::string& filename, const std::vector<std::string>& packageDirs);
    bool loadFromString(const std::string& sdfString, const std::vector<std::string>& packageDirs);

private:
    bool convertSDFormatToModel(const std::vector<std::string>& packageDirs);
};

#endif /* end of include guard: IDYNTREE_MODELIO_SDFORMAT_SDFORMATDOCUMENT_H \
        */
