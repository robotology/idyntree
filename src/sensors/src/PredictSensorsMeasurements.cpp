/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * email:  naveen.kuppuswamyt@iit.it
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

#include <iDynTree/Sensors/PredictSensorsMeasurements.hpp>

#include <iDynTree/Sensors/Sensors.hpp>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/ForwardKinematics.h>


// SensorList sensorList;
//         Traversal traversal;
//         Model model;

namespace iDynTree {
 
struct PredictSensorsMeasurements::PredictSensorsMeasurementsPrivateAttributes
{
        SensorsList sensorsList;
        Traversal traversal;
        Model model;
    
};
}