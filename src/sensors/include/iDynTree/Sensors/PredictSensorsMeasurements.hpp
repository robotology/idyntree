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


namespace iDynTree{
    
    class SensorsList;
    class Traversal;
    class Model;
    class VectorDynSize;

    class PredictSensorsMeasurements {
    
        
    public : 
        PredictSensorsMeasurements();
        bool setSensorList(const SensorsList &sensorsList);
        bool setModel(const iDynTree::Model &model);
        bool setTraversal(const iDynTree::Traversal &traversal);
        bool makePrediction(iDynTree::VectorDynSize &predictedMeasurement);
        ~PredictSensorsMeasurements();
        
    private :
        
        struct PredictSensorsMeasurementsPrivateAttributes;
        PredictSensorsMeasurementsPrivateAttributes * pimpl;
        
    };
}

#endif 