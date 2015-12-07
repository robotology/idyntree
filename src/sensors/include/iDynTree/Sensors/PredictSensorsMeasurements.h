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
#include "Sensors.hpp"


namespace iDynTree{
    
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

    /**
     * PredictSensorsMeasurements class.
     *
     * A computation of predicted measurements given some state
     *
     * \ingroup iDynTreeSensors
     *
     */
    class PredictSensorsMeasurements {
    
        
    public : 
      /**
       * Constructor.
      */
      PredictSensorsMeasurements();

      /**
       * Set predicted measurements into a reference to a SensorsMeasurements object
       * First version makes only Accelerometer and Gyroscop predictions
       * @return true if the sensors in the list are all valid
       */
       bool makePrediction(const iDynTree::Model & model,
                           const iDynTree::Traversal & traversal,
                           const iDynTree::FreeFloatingPos & robotPos,
                           const iDynTree::FreeFloatingVel & robotVel,
                           const iDynTree::FreeFloatingAcc & robotAcc,
                           const iDynTree::LinAcceleration & gravity,
                           const iDynTree::SensorsList &sensorsList,
                           iDynTree::SensorsMeasurements &predictedMeasurement);
      /**
       * Set predicted measurements into a reference to a VectorDynSize object reference
       * First version makes only Accelerometer and Gyroscope predictions
       * Ordering of the elements is in the manner mentioned in Nori, Kuppuswamy and Traversaro (2015)
       * @return true if the sensors in the list are all valid
       */
      bool makePrediction(const iDynTree::Model & model,
                           const iDynTree::Traversal & traversal,
                           const iDynTree::FreeFloatingPos & robotPos,
                           const iDynTree::FreeFloatingVel & robotVel,
                           const iDynTree::FreeFloatingAcc & robotAcc,
                           const iDynTree::LinAcceleration & gravity,
                           const iDynTree::SensorsList &sensorsList,
                           iDynTree::VectorDynSize &predictedMeasurement);
        ~PredictSensorsMeasurements();
        
    private :
        

    };
}

#endif 