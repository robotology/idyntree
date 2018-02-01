/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

/**
 * \file ModelSensorsTransformers.h
 *  \brief Collection of function to modify model and sensors in various ways
 *
 *
 */


#ifndef IDYNTREE_MODEL_SENSORS_TRANSFORMERS_H
#define IDYNTREE_MODEL_SENSORS_TRANSFORMERS_H

namespace iDynTree
{
class Model;
class SensorsList;

/**
 * Variant of createReducedModel function that also process the sensorList .
 *
 * Note
 */
bool createReducedModelAndSensors(const Model& fullModel,
                                  const SensorsList& fullSensors,
                                  const std::vector<std::string>& jointsInReducedModel,
                                        Model& reducedModel,
                                        SensorsList& reducedSensors);


}

#endif
