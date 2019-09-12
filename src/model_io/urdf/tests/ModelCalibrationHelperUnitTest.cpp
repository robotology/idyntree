/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/ModelIO/ModelCalibrationHelper.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

using namespace iDynTree;

double getTotalMass(const Model& model)
{
    double totalMass = 0.0;
    for(size_t l=0; l < model.getNrOfLinks(); l++)
    {
        totalMass += model.getLink(l)->getInertia().getMass();
    }

    return totalMass;
}

int main()
{
    // Open file with a single link with mass 1
    std::string urdfFileName = getAbsModelPath("oneLink.urdf");

    ModelCalibrationHelper mdlCalibHelper;
    bool ok = mdlCalibHelper.loadModelFromFile(urdfFileName);
    ASSERT_IS_TRUE(ok);

    // Check that the mass is one
    double expectedMass = 1.0;
    ASSERT_EQUAL_DOUBLE(getTotalMass(mdlCalibHelper.model()), expectedMass);

    // Modify the mass as an inertial paramters
    VectorDynSize inertialParams(10*mdlCalibHelper.model().getNrOfLinks());

    mdlCalibHelper.model().getInertialParameters(inertialParams);

    ASSERT_EQUAL_DOUBLE(inertialParams(0), expectedMass);
    double newMass = 2.0;
    inertialParams(0) = newMass;

    std::string newModel;
    ok = mdlCalibHelper.updateModelInertialParametersToString(newModel, inertialParams);
    ASSERT_IS_TRUE(ok);

    // Write to file for debug
    ok = mdlCalibHelper.updateModelInertialParametersToFile("ModelCalibrationHelperUnitTestModel.urdf", inertialParams);
    ASSERT_IS_TRUE(ok);

    std::cerr << newModel << std::endl;

    // Verify mass
    ModelLoader mdlLoader;

    ok = mdlLoader.loadModelFromString(newModel);
    ASSERT_IS_TRUE(ok);
    ASSERT_EQUAL_DOUBLE(getTotalMass(mdlLoader.model()), newMass);





    return EXIT_SUCCESS;
}
