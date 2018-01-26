/*!
 * @file  BoundingBoxHelpers.cpp
 * @author Jorhabib Eljaik
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2017
 */

#include <iDynTree/BoundingBoxHelpers.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <Eigen/Core>

#include <algorithm>

namespace iDynTree
{
    void BoundingBoxProjectionConstraint::setActive(const bool isActive)
    {
        m_isActive = isActive;
    }

    bool BoundingBoxProjectionConstraint::isActive()
    {
        return m_isActive;
    }

    size_t BoundingBoxProjectionConstraint::getNrOfConstraints()
    {
        return A.rows();
    }

    bool BoundingBoxProjectionConstraint::buildBoundingBox(const iDynTree::Direction xAxisOfPlaneInWorld,
                                                         const iDynTree::Direction yAxisOfPlaneInWorld,
                                                         const iDynTree::Position originOfPlaneInWorld,
                                                         const std::vector<Polygon> &supportPolygonsExpressedInSupportFrame,
                                                         const std::vector<Transform> &absoluteFrame_X_supportFrame)
    {
      o = originOfPlaneInWorld;
      toEigen(P).block<1, 3>(0,0) = toEigen(xAxisOfPlaneInWorld);
      toEigen(P).block<1, 3>(1,0) = toEigen(yAxisOfPlaneInWorld);

      // Transform the polygons in the absolute frame
      std::vector<Polygon> supportPolygonsExpressedInAbsoluteFrame;

      supportPolygonsExpressedInAbsoluteFrame.resize(supportPolygonsExpressedInSupportFrame.size());

      for(size_t i=0; i< supportPolygonsExpressedInSupportFrame.size(); i++)
      {
          supportPolygonsExpressedInAbsoluteFrame[i] = supportPolygonsExpressedInSupportFrame[i].applyTransform(absoluteFrame_X_supportFrame[i]);
      }

      // Project the polygons on the projection plane
      std::vector<Vector2> projectedPoints;

      for(size_t poly=0; poly < supportPolygonsExpressedInAbsoluteFrame.size(); poly++)
      {
          for(size_t vertex=0; vertex < supportPolygonsExpressedInAbsoluteFrame[poly].getNrOfVertices(); vertex++)
          {
               Vector2 newProjectedPoint = project(supportPolygonsExpressedInAbsoluteFrame[poly](vertex));
               //toEigen(newProjectedPoint) = toEigen(P)*toEigen(supportPolygonsExpressedInAbsoluteFrame[poly](vertex)-o);
               projectedPoints.push_back(newProjectedPoint);
          }
      }

      if (projectedPoints.size() <= 2)
      {
          return false;
      }

      // Compute the bounding box
      //  Build a vector from projectedPoints with the x coordinates
      std::vector<double> xCoords;
      std::vector<double> yCoords;
      for (auto point : projectedPoints) {
          xCoords.push_back(point(0));
          yCoords.push_back(point(1));
      }

      // Find min and max x coordinate
      auto minMaxXCoord = std::minmax_element(xCoords.begin(), xCoords.end());
      auto minMaxYCoord = std::minmax_element(yCoords.begin(), yCoords.end());
      double minX, maxX, minY, maxY;
      minX = *minMaxXCoord.first;
      maxX = *minMaxXCoord.second;
      minY = *minMaxYCoord.first;
      maxY = *minMaxYCoord.second;

      // Compute bounding box
      Vector2 vec1, vec2, vec3, vec4;
      vec1(0) = minX; vec1(1) = maxY;
      vec2(0) = maxX; vec2(1) = maxY;
      vec3(0) = maxX; vec3(1) = minY;
      vec4(0) = minX; vec4(1) = minY;

      projectedBoundingBox.m_vertices.resize(4);
      projectedBoundingBox.m_vertices.at(0) = vec1;
      projectedBoundingBox.m_vertices.at(1) = vec4;
      projectedBoundingBox.m_vertices.at(2) = vec3;
      projectedBoundingBox.m_vertices.at(3) = vec2;

      // Build the constraint matrix
      buildConstraintMatrix();
      return false;

      // Update AtimesP matrix
      AtimesP.resize(A.rows(),P.cols());

      toEigen(AtimesP) = toEigen(A)*toEigen(P);

      return true;
    }

    void BoundingBoxProjectionConstraint::buildConstraintMatrix()
    {
        // The rows of the A matrix and of the b vector depends on the number of vertices in the convex hull
        A.resize(projectedBoundingBox.getNrOfVertices(),2);
        b.resize(projectedBoundingBox.getNrOfVertices());

        // We assume that the vertices in projectedBoundingBox are expressed in counter-clockwise order: this
        // is ensured by the structure of the Monotone Chain algorithm

        // Iterate on all line segments one by one
        for (size_t i=0; i < projectedBoundingBox.getNrOfVertices(); i++)
        {
            Vector2 p0 = projectedBoundingBox(i);
            Vector2 p1;

            if ( i != projectedBoundingBox.getNrOfVertices()-1 )
            {
                p1 = projectedBoundingBox(i+1);
            }
            else
            {
                // if we are using the last vertix, we build the line connecting it with the first vertex
                p1 = projectedBoundingBox(0);
            }

            toEigen(A).block<1,2>(i,0) = Eigen::Vector2d(p1(1)-p0(1),p0(0)-p1(0));
            b(i) = p0(0)*p1(1) - p1(0)*p0(1);
        }

        return;
    }

    Vector2 BoundingBoxProjectionConstraint::project(Position& pos3dInAbsoluteFrame)
    {
        iDynTree::Vector2 projected;
        toEigen(projected) = toEigen(P) * toEigen(pos3dInAbsoluteFrame - o);
        return projected;
    }

    void BoundingBoxProjectionConstraint::setProjectionAlongDirection(Vector3 direction)
    {
        Vector3 xProjection, yProjection;

        // define the projection for the x-component
        xProjection.setVal(0, 1.0);
        xProjection.setVal(1, 0.0);
        xProjection.setVal(2, -direction.getVal(0) / direction.getVal(2));

        // define the projection for the y-component
        yProjection.setVal(0, 0.0);
        yProjection.setVal(1, 1.0);
        yProjection.setVal(2, -direction.getVal(1) / direction.getVal(2));

        // fill the projection matrix
        toEigen(Pdirection).block<1, 3>(0,0) = toEigen(xProjection);
        toEigen(Pdirection).block<1, 3>(1,0) = toEigen(yProjection);

    }

    Vector2 BoundingBoxProjectionConstraint::projectAlongDirection(iDynTree::Position& posIn3dInAbsoluteFrame)
    {
        iDynTree::Vector2 projected;
        toEigen(projected) = toEigen(Pdirection) * toEigen(posIn3dInAbsoluteFrame - o);
        return projected;
    }

}
