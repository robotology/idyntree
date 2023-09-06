// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * \file ModelSensorsTransformers.h
 *  \brief Collection of function to modify model and sensors in various ways
 *
 *
 */


#ifndef IDYNTREE_MODEL_SENSORS_TRANSFORMERS_H
#define IDYNTREE_MODEL_SENSORS_TRANSFORMERS_H

#ifdef __DEPRECATED
  #warning <iDynTree/ModelSensorsTransfomers.h> is deprecated. Please use <iDynTree/ModelTransfomers.h>. To disable this warning use -Wno-deprecated.
#endif

#include <iDynTree/Utils.h>


namespace iDynTree
{
class Model;
class SensorsList;

/**
 * Variant of createReducedModel function that also process the sensorList .
 *
 * Note: since iDynTree 10, this has been superseded by the regular createReducedModel
 * that also operated on the SensorsList associated with the model.
 * 
 * To migrate to this version, just change from:
 *
 * Model fullModel, reducedModel;
 * std::vector<std::string> jointsInReducedModel;
 * SensorsList fullSensors, reducedSensors;
 * 
 * bool ok = createReducedModelAndSensors(fullModel, fullSensors, jointsInReducedModel, reducedModel, reducedSensors);
 * 
 * // handle ok
 * // access reducedModel and reducedSensors
 * 
 * to
 *
 * Model fullModel, reducedModel;
 * std::vector<std::string> jointsInReducedModel;
 * SensorsList fullSensors;
 * fullModel.sensors() = fullSensors;
 * 
 * bool ok = createReducedModel(fullModel, jointsInReducedModel, reducedModel);
 *
 * // Handle ok
 * // access reducedModel and reducedModel.sensors()
 * 
 */
IDYNTREE_DEPRECATED_WITH_MSG("createReducedModelAndSensors is deprecated, please use the createReducedModel, see createReducedModelAndSensors documentation for more info")
bool createReducedModelAndSensors(const Model& fullModel,
                                  const SensorsList& fullSensors,
                                  const std::vector<std::string>& jointsInReducedModel,
                                        Model& reducedModel,
                                        SensorsList& reducedSensors);


}

#endif
