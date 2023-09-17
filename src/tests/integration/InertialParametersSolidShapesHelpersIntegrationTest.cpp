// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/TestUtils.h>

#include "testModels.h"

#include <iDynTree/InertialParametersSolidShapesHelpers.h>
#include <iDynTree/ModelLoader.h>


using namespace iDynTree;

int main()
{
    // Open file with a external mesh (cube centered in 0 and with vertices in (1,0,0), (0,0,1), etc etc
    std::string urdfFileName = getAbsModelPath("oneLink.urdf");

    // Load model
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(urdfFileName);

    ASSERT_IS_TRUE(ok);

    // Ensure that the model has one mesh
    Model model = mdlLoader.model();

    //ASSERT_IS_TRUE(model.collisionSolidShapes().linkSolidShapes[0].size() ==  1);

    // Extract the inertial parameters assuming a total mass of 24 Kg
    // The principal moment of inertia for a cube of 2 meters side will
    // be (density is 24/8 = 3 Kg/m^3) of 2*24/3 = 16 .
    // http://scienceworld.wolfram.com/physics/MomentofInertiaRectangularParallelepiped.html
    double totalMass = 24;
    double expectedInertia = 16;
    RotationalInertia rotInertia;
    rotInertia.zero();
    rotInertia(0, 0) = rotInertia(1, 1) = rotInertia(2, 2) = expectedInertia;


    VectorDynSize inertialParams;
    inertialParams.resize(10);
    inertialParams(2) = 3.0;
    ok = estimateInertialParametersFromLinkBoundingBoxesAndTotalMass(totalMass, model, inertialParams);
    ASSERT_IS_TRUE(ok);

    std::cerr << inertialParams.toString() << std::endl;

    // Update the inertial parameters of the model with the one
    ok = model.updateInertialParameters(inertialParams);
    ASSERT_IS_TRUE(ok);

    // Check the inertial params
    iDynTree::SpatialInertia inertia = model.getLink(0)->getInertia();

        std::cerr << inertia.getMass() << std::endl;
        std::cerr << totalMass << std::endl;

    ASSERT_EQUAL_DOUBLE(inertia.getMass(), totalMass);
    Position comZero;
    comZero.zero();
    ASSERT_EQUAL_VECTOR(inertia.getCenterOfMass(), comZero);
    std::cerr << inertia.getRotationalInertiaWrtFrameOrigin().toString() << std::endl;
    std::cerr << rotInertia.toString() << std::endl;

    ASSERT_EQUAL_MATRIX(inertia.getRotationalInertiaWrtFrameOrigin(), rotInertia);


    std::cerr << "Test successful." << std::endl;
    return EXIT_SUCCESS;
}
