/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Sensors/Sensors.h>
#include "testModels.h"
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/ModelIO/URDFDofsImport.h>

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/GyroscopeSensor.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
using namespace iDynTree;

void CheckMaterial(std::string fileName)
{
    std::cout<<"Tying to load model from URDF"<<std::endl;
    // load URDF model
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    ASSERT_IS_TRUE(ok);
    Model model = loader.model();
    std::cout<<"Model "<<fileName.c_str()<<" created with :"<<model.getNrOfDOFs()<<" DoFs"<<std::endl;
}



int main()
{
    std::cout<<"Read Model:\n";
    CheckMaterial(getAbsModelPath("/frame.urdf"));

    std::cout <<"Frame Material test just ran\n";
    return 0;
}
