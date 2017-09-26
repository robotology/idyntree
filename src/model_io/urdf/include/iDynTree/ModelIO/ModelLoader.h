/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_MODEL_LOADER_H
#define IDYNTREE_MODEL_LOADER_H

#include <iDynTree/Model/Model.h>
#include <iDynTree/Sensors/Sensors.h>

#include <string>
#include <vector>

namespace iDynTree
{

/**
 * \ingroup iDynTreeModelIO
 *
 * Helper class to load a model from a generic format.
 *
 */
class ModelLoader
{
private:
    Model m_model;
    SensorsList m_sensors;
    bool m_isModelValid;

    bool setModelAndSensors(const Model& _model, const SensorsList& _sensors);

public:

    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     *
     */
    ModelLoader();

    //@}

    /**
     * @name Model loading and definition methods
     * This methods are used to load the structure of your model.
     */
    //@{
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
     * Load reduced model from another model, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @note the order of the degreese of freedom of the newly loaded model
     * will be the one specified by the input joints serialization, i.e. consideredJoints
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] filetype (optional) explicit definition of the type of the loaded file. Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent), false otherwise.
     *
     * \note Until https://github.com/robotology/idyntree/issues/132 is fixed, this method does not
     *       accounts for sensors.
     *
     */
    bool loadReducedModelFromFullModel(const Model& fullModel,
                                       const std::vector<std::string> & consideredJoints,
                                       const std::string filetype="");

    /**
     * Load reduced model from string, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] modelString string containg the model of the robot.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent), false otherwise.
     *
     *
     */
    bool loadReducedModelFromString(const std::string modelString,
                                    const std::vector<std::string> & consideredJoints,
                                    const std::string filetype="");

    /**
     * Load reduced model from file, specifyng only the desired joints in the model.
     *
     * All other joints will be considered to be fixed to their default position,
     * and their child links will be lumped together.
     *
     * @param[in] filename path to the file to load.
     * @param[in] consideredJoints list of joints to consider in the model.
     * @param[in] filetype (optional) explicit definiton of the filetype to load.
     *                     Only "urdf" is supported at the moment.
     * @return true if all went well (files were correctly loaded and consistent), false otherwise.
     *
     *
     */
    bool loadReducedModelFromFile(const std::string filename,
                                  const std::vector<std::string> & consideredJoints,
                                  const std::string filetype="");

    /**
     * Get the loaded model.
     *
     */
    const Model & model();

    /**
     * Get the loaded sensors.
     */
    const SensorsList & sensors();

    /**
     * Return true if the model have been correctly true.
     *
     * @return True if the model was loaded correctly.
     */
    bool isValid();
    //@}
};

}

#endif
