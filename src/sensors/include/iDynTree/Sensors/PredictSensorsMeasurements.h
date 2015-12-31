/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * email:  naveen.kuppuswamy@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef PREDICTSENSORSMEASUREMENTS_HPP
#define PREDICTSENSORSMEASUREMENTS_HPP

namespace iDynTree
{

    class SensorsList;
    class Traversal;
    class Model;
    class VectorDynSize;
    class LinearMotionVector3;
    typedef LinearMotionVector3 LinAcceleration;
    class SensorsMeasurements;
    class FreeFloatingAcc;
    class FreeFloatingPos;
    class FreeFloatingVel;
    class SensorsMeasurement;
    class LinkPositions;
    class LinkVelArray;
    class LinkAccArray;


    /**
     * \brief Predict the measurement of a set of sensors.
     *
     * Given a SensorList object describing a list of sensor of a
     * model fill the output argument predictedMeasurement
     * with the predicted measurement of the sensors consistent with the state
     * and the acceleration/torques and forces of the Model.
     *
     * At the moment, only Accelerometers and Gyroscopes sensors are
     * handled by this function.
     *
     * This function takes in input the internal buffers used for the
     * computation to avoid dynamic memory allocation.
     *
     * \ingroup iDynTreeSensors
     *
     * @param[in] model the model used to predict the sensor measurements.
     * @param[in] sensorList the sensors list used to predict the sensors measurements.
     * @param[in] traversal the Traversal used for predict the sensor measurements.
     * @param[in] robotpos the position of the model used for prediction.
     * @param[in] robotvel the velocity of the model used for prediction.
     * @param[in] robotacc the acceleration of the model used for prediction.
     * @param[in] gravity the gravity acceleration (in world frame) used for prediction.
     * @param[out] buf_properRobotAcc internal buffer, storing the proper acceleration of the model.
     * @param[out] buf_linkPos internal buffer, storing the position of every link in the model.
     * @param[out] buf_linkVel internal buffer, storing the velocity of every link in the model.
     * @param[out] buf_linkProperAcc internal buffer, storing the proper acceleration of every link in the model.
     * @param[out] predictedMeasurement the predicted measurements for the sensors.
     *
     * @return true if the sensors in the list are all valid
     */
     bool predictSensorsMeasurements(const iDynTree::Model & model,
                                     const iDynTree::SensorsList &sensorsList,
                                     const iDynTree::Traversal & traversal,
                                     const iDynTree::FreeFloatingPos& robotPos,
                                     const iDynTree::FreeFloatingVel& robotVel,
                                     const iDynTree::FreeFloatingAcc& robotAcc,
                                     const iDynTree::LinAcceleration & gravity,
                                           iDynTree::FreeFloatingAcc& buf_properRobotAcc,
                                           iDynTree::LinkPositions& buf_linkPos,
                                           iDynTree::LinkVelArray& buf_linkVel,
                                           iDynTree::LinkAccArray& buf_linkProperAcc,
                                           iDynTree::SensorsMeasurements &predictedMeasurement);

}

#endif