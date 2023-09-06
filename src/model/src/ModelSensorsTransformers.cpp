// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Model.h>
#include <iDynTree/ModelTransformers.h>

#include <iDynTree/Sensors.h>
#include <iDynTree/ModelSensorsTransformers.h>

#include <iDynTree/SixAxisForceTorqueSensor.h>


#include <cassert>
#include <set>


namespace iDynTree
{

bool createReducedModelAndSensors(const Model& fullModel,
                                  const SensorsList& fullSensors,
                                  const std::vector<std::string>& jointsInReducedModel,
                                        Model& reducedModel,
                                        SensorsList& reducedSensors)
{
    Model fullModelCopy = fullModel;
    fullModelCopy.sensors() = fullSensors;

    if (!createReducedModel(fullModelCopy, jointsInReducedModel, reducedModel)) {
        return false;
    }

    reducedSensors = reducedModel.sensors();

    return true;
}


}
