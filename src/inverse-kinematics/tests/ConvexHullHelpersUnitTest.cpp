// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ConvexHullHelpers.h>

#include <iDynTree/Axis.h>
#include <iDynTree/TestUtils.h>

void testConvexHullProjectionConstraint()
{
    // Check that given an "easy" problem the constraint works fine
    iDynTree::ConvexHullProjectionConstraint projectionConstraint;

    // Add a nice big square convex hull
    iDynTree::Polygon notProjectedConvexHull;
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position( 1.0, 1.0, 5.0));
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position(-1.0, 1.0, 10.0));
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position(-1.0, -1.0, -10.0));
    notProjectedConvexHull.m_vertices.push_back(iDynTree::Position( 1.0, -1.0, -15.0));

    std::vector<iDynTree::Polygon> polygons;
    polygons.resize(1);
    polygons[0] = notProjectedConvexHull;

    // Add a smaller polygon inside the other one
    iDynTree::Polygon notProjectedConvexHullSmall;
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position( 0.5, 0.5, 5.0));
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position(-0.5, 0.5, 10.0));
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position(-0.5, -0.5, -10.0));
    notProjectedConvexHullSmall.m_vertices.push_back(iDynTree::Position( 0.5, -0.5, -15.0));

    polygons.push_back(notProjectedConvexHullSmall);

    std::vector<iDynTree::Transform> transforms;
    transforms.push_back(iDynTree::Transform::Identity());
    transforms.push_back(iDynTree::Transform::Identity());


    iDynTree::Direction xAxis(1.0, 0.0, 0.0);
    iDynTree::Direction yAxis(0.0, 1.0, 0.0);

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

void testConvexHullProjectionWithGravity()
{
    // For this test, we will use a simple convex hull of side 1 meter
    // centered in the origin of the world and laying on the XY plane
    iDynTree::ConvexHullProjectionConstraint projectionConstraint;
    iDynTree::Polygon convexHull;
    convexHull.m_vertices.push_back(iDynTree::Position( 0.5, 0.5, 0.0));
    convexHull.m_vertices.push_back(iDynTree::Position(-0.5, 0.5, 0.0));
    convexHull.m_vertices.push_back(iDynTree::Position(-0.5, -0.5, 0.0));
    convexHull.m_vertices.push_back(iDynTree::Position( 0.5, -0.5, 0.0));
    std::vector<iDynTree::Polygon> polygons;
    polygons.resize(1);
    polygons[0] = convexHull;


    // The convex hull is already expressed in the world
    std::vector<iDynTree::Transform> transforms;
    transforms.push_back(iDynTree::Transform::Identity());

    iDynTree::Direction xAxis(1.0, 0.0, 0.0);
    iDynTree::Direction yAxis(0.0, 1.0, 0.0);

    bool ok = projectionConstraint.buildConvexHull(xAxis,yAxis,iDynTree::Position::Zero(),polygons,transforms);
    ASSERT_IS_TRUE(ok);

    // Use a test position that is outside the convex hull with the normal projection
    iDynTree::Position testPoint(1.0, 0.0, 1.0);

    // Projected along the normal of the plane, this point is outside the convex hull
    // Note: positive margin means inside, negative outside
    ASSERT_IS_TRUE(projectionConstraint.computeMargin(projectionConstraint.project(testPoint)) < 0);

    //----------------------------------------------------------------------------------------------
    // If the direction along which we are projecting is the normal of the plane, the projection
    // should match the one done without direction
    iDynTree::Direction planeNormal(0.0, 0.0, -1.0);
    projectionConstraint.setProjectionAlongDirection(planeNormal);
    ASSERT_EQUAL_VECTOR(projectionConstraint.project(testPoint), projectionConstraint.projectAlongDirection(testPoint));

    //----------------------------------------------------------------------------------------------
    // If the direction of the projection is "skewed" to the left, the point should instead be inside the convex hull
    iDynTree::Direction skewedDirection(-1.0, 0.0, -1.0);

    projectionConstraint.setProjectionAlongDirection(skewedDirection);
    std::cerr << projectionConstraint.computeMargin(projectionConstraint.projectAlongDirection(testPoint)) << std::endl;

    ASSERT_IS_TRUE(projectionConstraint.computeMargin(projectionConstraint.projectAlongDirection(testPoint)) > 0);

    //----------------------------------------------------------------------------------------------
    // The code should work even if I try to project in the opposite direction wrt 'planeNormal'
    iDynTree::Direction upwardDirection(0.0, 0.0, 1.0);

    projectionConstraint.setProjectionAlongDirection(upwardDirection);

    // I should obtain the same result as the orthogonal projection
    ASSERT_EQUAL_VECTOR(projectionConstraint.project(testPoint), projectionConstraint.projectAlongDirection(testPoint));
    
    // The projected point should be again outside the convex hull
    ASSERT_IS_TRUE(projectionConstraint.computeMargin(projectionConstraint.projectAlongDirection(testPoint))< 0);

}

int main()
{
    testConvexHullProjectionConstraint();
    testConvexHullProjectionWithGravity();

    return EXIT_SUCCESS;
}
