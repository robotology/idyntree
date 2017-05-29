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

    // Process first F/T sensors
    for(size_t sens=0; sens < fullSensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE); sens++)
    {
        SixAxisForceTorqueSensor* pSens = static_cast<SixAxisForceTorqueSensor*>(fullSensors.getSensor(SIX_AXIS_FORCE_TORQUE,sens));
        std::string parentJointName = pSens->getParentJoint();


        // If the sensor at which the sensor is attached is not in the reduced model, drop the sensor
        if( reducedModel.isJointNameUsed(parentJointName) )
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

    // Then all link sensors
    for (SensorsList::const_iterator it = fullSensors.allSensorsIterator(); it.isValid(); ++it)
    {
        Sensor *s = *it;


        // This should select only link sensors
        LinkSensor *linkSens = dynamic_cast<LinkSensor*>(s);
        if( linkSens )
        {
            std::string sensorLinkInFullModel = linkSens->getParentLink();

            // If the link to wicht the sensors is attached (parentLink) is also in the reduced model, we can just copy the sensor to the reduced sensors models
            if( reducedModel.isLinkNameUsed(sensorLinkInFullModel) )
            {
                // update the link index of the reduced model
                LinkSensor *sensorInReducedModel = static_cast<LinkSensor*>(linkSens->clone());
                if (!sensorInReducedModel || !sensorInReducedModel->updateIndeces(reducedModel)) {
                    reportError("","createReducedModelAndSensors", "Failed to duplicate LinkSensor and update indeces");
                    return false;
                }
                reducedSensors.addSensor(*sensorInReducedModel);
                delete sensorInReducedModel;
            }
            else
            {
                // Otherwise there should be a additional frame in the reduced model named like the sensor link in the full model
                if( !reducedModel.isFrameNameUsed(sensorLinkInFullModel) )
                {
                    std::stringstream ss;
                    ss << "additional frame " << sensorLinkInFullModel << " is not in the reduced model, reducing sensors failed" << std::endl;
                    reportError("","createReducedModelAndSensors",ss.str().c_str());
                    return false;
                }


                FrameIndex sensorLinkAdditionalFrameIndexInReducedModel = reducedModel.getFrameIndex(sensorLinkInFullModel);
                LinkIndex sensorLinkInReducedModelIdx = reducedModel.getFrameLink(sensorLinkAdditionalFrameIndexInReducedModel);

                std::string sensorLinkInReducedModel = reducedModel.getLinkName(sensorLinkInReducedModelIdx);

                // If we found the original link sensor as an additonal frame, we can compute the pose of the sensor w.r.t. to the new link to which it is attached
                Transform sensorLinkInReducedModel_H_sensorLinkInFullModel = reducedModel.getFrameTransform(sensorLinkAdditionalFrameIndexInReducedModel);
                Transform sensorLinkInFullModel_H_sensorFrame              = linkSens->getLinkSensorTransform();

                Transform sensorLinkInReducedModel_H_sensorFrame = sensorLinkInReducedModel_H_sensorLinkInFullModel*sensorLinkInFullModel_H_sensorFrame;

                // Copy the sensor to modify it
                LinkSensor* sensorCopy = (LinkSensor*)linkSens->clone();

                // Update the pose
                sensorCopy->setLinkSensorTransform(sensorLinkInReducedModel_H_sensorFrame);

                // Update the link name and indeces
                sensorCopy->setParentLink(sensorLinkInReducedModel);
                sensorCopy->setParentLinkIndex(reducedModel.getLinkIndex(sensorLinkInReducedModel));

                reducedSensors.addSensor(*sensorCopy);

                delete sensorCopy;
            }

        }
    }

    return ok;
}


}
