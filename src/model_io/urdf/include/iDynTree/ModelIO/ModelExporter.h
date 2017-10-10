/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_MODEL_EXPORTER_H
#define IDYNTREE_MODEL_EXPORTER_H

#include <iDynTree/Model/Model.h>
#include <iDynTree/Sensors/Sensors.h>

#include <memory>
#include <string>
#include <vector>

namespace iDynTree
{

/**
 * \ingroup iDynTreeModelIO
 *
 * Options for the iDynTree exporter.
 */
class ModelExporterOptions
{
public:
    /**
     * Specify the base link of the exported model.
     *
     * Differently from the iDynTree::Model class, some file formats (such as the URDF) represent the multibody structure
     * using a directed tree representation, for which it is necessary to explicitly specify a base link.
     *
     * If this string is empty (default value), the default base link contained in the model will be used.
     *
     * Default value: "".
     * Supported formats: urdf.
     */
    std::string baseLink;

    /**
     * Constructor.
     */
    ModelExporterOptions();

};


/**
 * \ingroup iDynTreeModelIO
 *
 * Helper class to export a model to the supported textual formats.
 *
 * Currently the only format supported for export is the URDF format,
 * as it is described in http://wiki.ros.org/urdf/XML .
 *
 * Only iDynTree::Model classes that represent multibody system with no loops
 * can be exported.
 *
 * Furthermore, currently the model exporter only exports a subset of the features
 * supported in iDynTree::Model. In particular, the following features are not exported:
 * * Additional frames
 * * Visual and collision geometries
 * * Sensors
 * * Joint limits, damping and static friction.
 *
 *
 */
class ModelExporter
{
private:

    class Pimpl;
    std::unique_ptr<Pimpl> m_pimpl;

public:

    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     */
    ModelExporter();

    ~ModelExporter();

    //@}

    /**
     * @name Model exporting and definition methods
     * This methods are used to export the structure of your model.
     */
    //@{
    const ModelExporterOptions& exportingOptions() const;

    void setExportingOptions(const ModelExporterOptions& options);

    /**
     * Specifies the model of the robot to export.
     *
     * @param[in] model The used model.
     * @param[in] sensors The used sensors.
     * @param[in] options The used options.
     * @return true if all went well, false otherwise.
     */
    bool init(const Model& model,
              const SensorsList& sensors=SensorsList(),
              const ModelExporterOptions options=ModelExporterOptions());

    /**
     * Get the loaded model that will be exported.
     *
     */
    const Model & model();

    /**
     * Get the loaded sensors that will be exported.
     */
    const SensorsList & sensors();

    /**
     * Return true if the model have been correctly loaded, and can be exported.
     *
     * @return True if the model was loaded correctly.
     */
    bool isValid();

    /**
     * Export the model of the robot to a string.
     *
     * @param modelString string containg the model of the robot.
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool exportModelToString(std::string & modelString, const std::string filetype="urdf");

    /**
     * Export the model of the robot to an external file.
     *
     * @param filename path to the file to export.
     *                 It can be either a relative filename with respect to the current working directory,
     *                 or an absolute filename.
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool exportModelToFile(const std::string & filename, const std::string filetype="urdf");
    //@}
};

}

#endif
