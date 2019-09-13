/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_MODEL_CALIBRATION_HELPER_H
#define IDYNTREE_MODEL_CALIBRATION_HELPER_H

#include <iDynTree/Model/Model.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/ModelIO/ModelExporter.h>

#include <memory>
#include <string>
#include <vector>

namespace iDynTree
{


/**
 * \ingroup iDynTreeModelIO
 *
 * Helper class to load a model, modify its parameters based on calibration,
 * and save it again to file.
 *
 */
class ModelCalibrationHelper
{
private:
    class ModelCalibrationHelperPimpl;
    std::unique_ptr<ModelCalibrationHelperPimpl> m_pimpl;

public:

    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     *
     */
    ModelCalibrationHelper();

    ~ModelCalibrationHelper();

    //@}

    /**
     * Load the model of the robot  from a string.
     *
     * @param modelString string containg the model of the robot.
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool loadModelFromString(const std::string & modelString, const std::string & filetype="urdf");

    /**
     * Load the model of the robot from an external file.
     *
     * @param filename path to the file to load
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool loadModelFromFile(const std::string & filename, const std::string & filetype="urdf");

    /**
     * Update the inertial parameters of the loaded model.
     *
     * @note the inertial params vector follow the structure of the Model::getInertialParameters method.
     */
    bool updateModelInertialParametersToString(std::string & modelString, 
                                               const iDynTree::VectorDynSize& inertialParams,
                                               const std::string filetype="urdf",
                                               const iDynTree::ModelExporterOptions options=iDynTree::ModelExporterOptions());

    /**
     * Update the inertial parameters of the loaded model.
     *
     * @note the inertial params vector follows the structure of the Model::getInertialParameters method.
     */
    bool updateModelInertialParametersToFile(const std::string & filename, 
                                             const iDynTree::VectorDynSize& inertialParams,
                                             const std::string filetype="urdf",
                                             const iDynTree::ModelExporterOptions options=iDynTree::ModelExporterOptions());

    /**
     * Get the loaded model.
     *
     * @note This always return the model loaded via loadModel methods, and is not affected by the updateModel methods.
     */
    const Model & model();

    /**
     * Get the loaded sensors.
     * 
     * @note This always return the model loaded via loadModel method, and is not affected by the updateModel methods.
     */
    const SensorsList & sensors();

    /**
     * Return true if the model have been correctly true.
     *
     * @note This always return the validity of the model loaded via loadModel method, and is not affected by the updateModel methods.
     * @return True if the model was loaded correctly.
     */
    bool isValid();
    //@}
};

}

#endif
