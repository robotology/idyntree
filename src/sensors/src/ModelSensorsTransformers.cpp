/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ModelTransformers.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/ModelSensorsTransformers.h>

#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>


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
    if (!createReducedModel(fullModel, jointsInReducedModel, reducedModel)) {
        return false;
    }

    // make sure that reducedSensors is empty
    assert(reducedSensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE) == 0);
    assert(reducedSensors.getNrOfSensors(ACCELEROMETER) == 0);
    assert(reducedSensors.getNrOfSensors(GYROSCOPE) == 0);


    // Process first F/T sensors
    for (auto it = fullSensors.sensorsIteratorForType(SIX_AXIS_FORCE_TORQUE); it.isValid(); ++it) {
        Sensor* s = *it;
        JointSensor* jointSens = dynamic_cast<JointSensor*>(s);

        // If the sensor is a joint sensor
        if (jointSens) {
            // The parent joint can be present in the reduced model, or it could have been assigned to
            // a submodel after the reduction
            std::string parentJointName = jointSens->getParentJoint();

            // If the parent's joint is present in the model
            if (reducedModel.isJointNameUsed(parentJointName)) {
                // If we add the sensor to the new sensors list, we have to upgrade the indices
                SixAxisForceTorqueSensor* sensorCopy;
                sensorCopy = static_cast<SixAxisForceTorqueSensor*>(jointSens->clone());

                std::string oldFirstLinkName = sensorCopy->getFirstLinkName();
                std::string oldSecondLinkName = sensorCopy->getSecondLinkName();

                iDynTree::LinkIndex firstLinkIndex = reducedModel.getLinkIndex(oldFirstLinkName);
                iDynTree::LinkIndex secondLinkIndex = reducedModel.getLinkIndex(oldSecondLinkName);

                // The reduced model contains both the links attached to the joint
                // No particular operations are required in this case
                if ((firstLinkIndex != iDynTree::LINK_INVALID_INDEX) &&
                    (secondLinkIndex != iDynTree::LINK_INVALID_INDEX)) {
                    // Update indices
                    sensorCopy->updateIndices(reducedModel);
                    // Add the sensor to the reduced model
                    reducedSensors.addSensor(*sensorCopy);
                }
                // If one of the links attached to the joint has been lumped in the reduced model,
                // updating the transform is required
                else if (firstLinkIndex != iDynTree::LINK_INVALID_INDEX) {
                    // The secondLink has been lumped
                    // Get the link to which it was merged
                    FrameIndex frameIndexOfSecondLink = reducedModel.getFrameIndex(oldSecondLinkName);
                    LinkIndex newSecondLinkIndex = reducedModel.getFrameLink(frameIndexOfSecondLink);

                    // Update the transform. It requires two steps:
                    // New second link (reducedModel) -> Old second link (fullModel) -> jointSens frame
                    Transform oldSecondLinkInFullModel_H_sensorFrame;

                    sensorCopy->getLinkSensorTransform(sensorCopy->getSecondLinkIndex(),
                                                       oldSecondLinkInFullModel_H_sensorFrame);

                    Transform newSecondLinkInReducedModel_H_oldSecondLinkInFullModel =
                        reducedModel.getFrameTransform(frameIndexOfSecondLink);

                    // Get the name of the new second link (to which the old one has been lumped)
                    std::string newSecondLinkName = reducedModel.getLinkName(newSecondLinkIndex);

                    // Set the name of the new secondLink and update its index
                    sensorCopy->setSecondLinkName(newSecondLinkName);
                    sensorCopy->updateIndices(reducedModel);

                    // Update the transform
                    sensorCopy->setSecondLinkSensorTransform(sensorCopy->getSecondLinkIndex(),
                                                            newSecondLinkInReducedModel_H_oldSecondLinkInFullModel*oldSecondLinkInFullModel_H_sensorFrame);

                    // Update the appliedWrenchLink
                    if (fullModel.getLinkName(sensorCopy->getAppliedWrenchLink()) == oldSecondLinkName) {
                        sensorCopy->setAppliedWrenchLink(reducedModel.getLinkIndex(newSecondLinkName));
                    }

                    // Add the sensor to the reduced model
                    reducedSensors.addSensor(*sensorCopy);
                }
                else if (secondLinkIndex != iDynTree::LINK_INVALID_INDEX) {
                    // The firstLink has been lumped
                    // Get the link to which it was merged
                    FrameIndex frameIndexOfFirstLink = reducedModel.getFrameIndex(oldFirstLinkName);
                    LinkIndex newFirstLinkIndex = reducedModel.getFrameLink(frameIndexOfFirstLink);

                    // Update the transform. It requires two steps:
                    // New first link (reducedModel) -> Old first link (fullModel) -> jointSens frame
                    Transform oldFirstLinkInFullModel_H_sensorFrame;

                    sensorCopy->getLinkSensorTransform(sensorCopy->getFirstLinkIndex(),
                                                       oldFirstLinkInFullModel_H_sensorFrame);

                    Transform newFirstLinkInReducedModel_H_oldFirstLinkInFullModel =
                        reducedModel.getFrameTransform(frameIndexOfFirstLink);

                    // Get the name of the new first link (to which the old one has been lumped)
                    std::string newFirstLinkName = reducedModel.getLinkName(newFirstLinkIndex);

                    // Set the name of the new firstLink and update its index
                    sensorCopy->setFirstLinkName(newFirstLinkName);
                    sensorCopy->updateIndices(reducedModel);

                    // Update the transform
                    sensorCopy->setFirstLinkSensorTransform(sensorCopy->getFirstLinkIndex(),
                                                            newFirstLinkInReducedModel_H_oldFirstLinkInFullModel*oldFirstLinkInFullModel_H_sensorFrame);

                    // Update the appliedWrenchLink
                    if (fullModel.getLinkName(sensorCopy->getAppliedWrenchLink()) == oldFirstLinkName) {
                        sensorCopy->setAppliedWrenchLink(reducedModel.getLinkIndex(newFirstLinkName));
                    }

                    // Add the sensor to the reduced model
                    reducedSensors.addSensor(*sensorCopy);
                }
                else {
                    std::stringstream ss;
                    ss << "The links related to the joint sensor attached on " << parentJointName << " have an invalid index" << std::endl;
                    reportError("", "createReducedModelAndSensors", ss.str().c_str());
                    delete sensorCopy;
                    return false;
                }

                delete sensorCopy;
            }
        }
        else {
            std::stringstream ss;
            ss << "The processed FT sensor couldn't be cast as a joint sensor" << std::endl;
            reportWarning("", "createReducedModelAndSensors", ss.str().c_str());

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
                if (!sensorInReducedModel || !sensorInReducedModel->updateIndices(reducedModel)) {
                    reportError("","createReducedModelAndSensors", "Failed to duplicate LinkSensor and update indices");
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

                // Update the link name and indices
                sensorCopy->setParentLink(sensorLinkInReducedModel);
                sensorCopy->setParentLinkIndex(reducedModel.getLinkIndex(sensorLinkInReducedModel));

                reducedSensors.addSensor(*sensorCopy);

                delete sensorCopy;
            }

        }
    }

    return true;
}


}
