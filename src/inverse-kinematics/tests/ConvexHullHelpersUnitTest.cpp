/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/ConvexHullHelpers.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/TestUtils.h>

void testConvexHullProjectionConstraint()
{
    // Check that given an "easy" problem the constraint works fine
    iDynTree::ConvexHullProjectionConstraint projectionConstraint;

    // Add a nice big square convex hull
    iDynTree::Polygon notProjectedConvexHull;
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position( 1.0, 1.0,5.0));
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position(-1.0, 1.0,10.0));
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position(-1.0,-1.0,-10.0));
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position( 1.0,-1.0,-15.0));

    std::vector<iDynTree::Polygon> polygons;
    polygons.resize(1);
    polygons[0] = notProjectedConvexHull;

    // Add a smaller polygon inside the other one
    iDynTree::Polygon notProjectedConvexHullSmall;
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position( 0.5, 0.5,5.0));
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position(-0.5, 0.5,10.0));
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position(-0.5,-0.5,-10.0));
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position( 0.5,-0.5,-15.0));

    polygons.push_back(notProjectedConvexHullSmall);

    std::vector<iDynTree::Transform> transforms;
    transforms.push_back(iDynTree::Transform::Identity());
    transforms.push_back(iDynTree::Transform::Identity());


    iDynTree::Direction xAxis(1.0,0.0,0.0);
    iDynTree::Direction yAxis(0.0,1.0,0.0);

    bool ok = projectionConstraint.buildConvexHull(xAxis,yAxis,iDynTree::Position::Zero(),polygons,transforms);

    ASSERT_IS_TRUE(ok);

    ASSERT_IS_TRUE(projectionConstraint.projectedConvexHull.getNrOfVertices() == 4);

    // First point
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(0)(0), -1.0);
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(0)(1), -1.0);

    // Second point
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(1)(0), 1.0);
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(1)(1),-1.0);

    // Third point
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(2)(0), 1.0);
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(2)(1), 1.0);

    // Fourth point
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(3)(0),-1.0);
    ASSERT_EQUAL_DOUBLE(projectionConstraint.projectedConvexHull(3)(1), 1.0);

}

int main()
{
    testConvexHullProjectionConstraint();

    return EXIT_SUCCESS;
}
