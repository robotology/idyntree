// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/TestUtils.h>


#include <iDynTree/InertialParametersSolidShapesHelpers.h>

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/Model.h>


using namespace iDynTree;

void checkOneCubeVsEightSmallCubes()
{
    // Create a model with one shape with one cube of 1 meter of side,
    // and another with 8 with 1/2 meter of side, placed to form a bigger cube
    // As the density is linear, verify that given the same mass the estimated inertial parameters are
    // the same

    iDynTree::Model oneCubeModel;
    iDynTree::Link oneCubeLink;
    oneCubeModel.addLink("link0", oneCubeLink);
    oneCubeModel.collisionSolidShapes().getLinkSolidShapes()[0].resize(1);
    iDynTree::Box oneCube;
    oneCube.setLink_H_geometry(Transform::Identity());
    oneCube.setX(1.0);
    oneCube.setY(1.0);
    oneCube.setZ(1.0);
    oneCubeModel.collisionSolidShapes().getLinkSolidShapes()[0][0] = new iDynTree::Box(oneCube);

    iDynTree::Model eightCubeModel;
    iDynTree::Link eightCubeLink;
    eightCubeModel.addLink("link0", eightCubeLink);
    oneCubeModel.collisionSolidShapes().getLinkSolidShapes()[0];

    iDynTree::Box smallCube;
    smallCube.setLink_H_geometry(Transform::Identity());
    smallCube.setX(1.0/2.0);
    smallCube.setY(1.0/2.0);
    smallCube.setZ(1.0/2.0);

    // Generate origins for the small cubes
    std::vector<Position> smallCubesOrigins;
    // + + +
    smallCubesOrigins.push_back(Position(+1.0/4.0, +1.0/4.0, +1.0/4.0));

    // + + -
    smallCubesOrigins.push_back(Position(+1.0/4.0, +1.0/4.0, -1.0/4.0));

    // + - +
    smallCubesOrigins.push_back(Position(+1.0/4.0, -1.0/4.0, +1.0/4.0));

    // + - -
    smallCubesOrigins.push_back(Position(+1.0/4.0, -1.0/4.0, -1.0/4.0));

    // - + +
    smallCubesOrigins.push_back(Position(-1.0/4.0, +1.0/4.0, +1.0/4.0));

    // - + -
    smallCubesOrigins.push_back(Position(-1.0/4.0, +1.0/4.0, -1.0/4.0));

    // - - +
    smallCubesOrigins.push_back(Position(-1.0/4.0, -1.0/4.0, +1.0/4.0));

    // - - -
    smallCubesOrigins.push_back(Position(-1.0/4.0, -1.0/4.0, -1.0/4.0));

    for(auto&& smallCubeOrigin: smallCubesOrigins) {
        smallCube.setLink_H_geometry(Transform(Rotation::Identity(), smallCubeOrigin));
        eightCubeModel.collisionSolidShapes().getLinkSolidShapes()[0].push_back(new Box(smallCube));
    }

    // Compute the inertia with the bounding boxes for both cases
    double totalMass = 1.0;
    VectorDynSize oneCubeParams(oneCubeModel.getNrOfLinks()*10), eightCubesParams(oneCubeModel.getNrOfLinks()*10);
    bool ok = estimateInertialParametersFromLinkBoundingBoxesAndTotalMass(totalMass,
                                                                          oneCubeModel,
                                                                          oneCubeParams);
    ASSERT_IS_TRUE(ok);

    ok = estimateInertialParametersFromLinkBoundingBoxesAndTotalMass(totalMass,
                                                                     eightCubeModel,
                                                                     eightCubesParams);
    ASSERT_IS_TRUE(ok);

    ASSERT_EQUAL_VECTOR(oneCubeParams, eightCubesParams);

    std::cerr << "oneCubeParams: " << oneCubeParams.toString() << std::endl;
    std::cerr << "eightCubesParams: " << eightCubesParams.toString() << std::endl;

    // The first parameter is the mass
    ASSERT_EQUAL_DOUBLE(oneCubeParams(0), totalMass);
}


int main()
{
    checkOneCubeVsEightSmallCubes();

    return EXIT_SUCCESS;
}

