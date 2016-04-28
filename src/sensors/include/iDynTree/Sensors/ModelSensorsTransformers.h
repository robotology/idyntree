/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
 *
 */
bool createReducedModelAndSensors(const Model& fullModel,
                                  const SensorsList& fullSensors,
                                  const std::vector<std::string>& jointsInReducedModel,
                                        Model& reducedModel,
                                        SensorsList& reducedSensors);


}

#endif