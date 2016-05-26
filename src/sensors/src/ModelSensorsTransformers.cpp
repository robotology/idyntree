/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ModelTransformers.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/ModelSensorsTransformers.h>

#include <iDynTree/Sensors/SixAxisFTSensor.h>


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
    bool ok = createReducedModel(fullModel,jointsInReducedModel,reducedModel);

    // make sure that reducedSensors is empty
    assert(reducedSensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE) == 0);

    if( !ok ) return false;

    // \todo currently only FT sensors are processed, extend to all sensors
    for(size_t sens=0; sens < fullSensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE); sens++)
    {
        SixAxisForceTorqueSensor* pSens = static_cast<SixAxisForceTorqueSensor*>(fullSensors.getSensor(SIX_AXIS_FORCE_TORQUE,sens));
        std::string parentJointName = pSens->getParentJoint();
        iDynTree::JointIndex parentJointIndex = reducedModel.getJointIndex(parentJointName);

        // If the sensor at which the sensor is attached is not in the reduced model, drop the sensor
        if( parentJointIndex != iDynTree::JOINT_INVALID_INDEX )
        {
            // If we add the sensor to the new sensors list, we have to upgrade the indeces
            SixAxisForceTorqueSensor* sensorCopy = (SixAxisForceTorqueSensor*)pSens->clone();

            // For now we assume that the two links at which the FT sensors is attached are not reduced.
            // A more advanced version of this function could properly handle that case \todo TODO
            std::string firstLinkName  = sensorCopy->getFirstLinkName();
            std::string secondLinkName = sensorCopy->getSecondLinkName();

            iDynTree::LinkIndex firstLinkIndex = reducedModel.getLinkIndex(firstLinkName);
            if( firstLinkIndex == iDynTree::LINK_INVALID_INDEX )
            {
                std::cerr << "[ERROR] createReducedModelAndSensors : " << firstLinkName << " is not in the reduced model, reducing sensors failed" << std::endl;
                return false;
            }

            iDynTree::LinkIndex secondLinkIndex = reducedModel.getLinkIndex(secondLinkName);
            if( secondLinkIndex == iDynTree::LINK_INVALID_INDEX )
            {
                std::cerr << "[ERROR] createReducedModelAndSensors : " << secondLinkName << " is not in the reduced model, reducing sensors failed" << std::endl;
                return false;
            }

            // Update indeces
            sensorCopy->updateIndeces(reducedModel);

            reducedSensors.addSensor(*sensorCopy);

            delete sensorCopy;
        }
    }

    return ok;
}


}
