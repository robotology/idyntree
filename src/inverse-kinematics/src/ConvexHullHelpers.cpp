/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/ConvexHullHelpers.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <Eigen/Core>

#include <algorithm>

namespace iDynTree
{
    Polygon::Polygon(): m_vertices(0)
    {

    }

    size_t Polygon::getNrOfVertices() const
    {
        return m_vertices.size();
    }

    void Polygon::setNrOfVertices(size_t size)
    {
        m_vertices.resize(size);
        return;
    }

    bool Polygon::isValid() const
    {
        return (m_vertices.size() >= 3);
    }

    Polygon Polygon::applyTransform(const Transform &newFrame_X_oldFrame) const
    {
        Polygon ret;

        assert(this->isValid());

        ret.setNrOfVertices(this->getNrOfVertices());

        for (size_t i=0; i < this->getNrOfVertices(); i++)
        {
            ret(i) = newFrame_X_oldFrame*this->operator()(i);
        }

        return ret;
    }

    Position & Polygon::operator()(const size_t idx)
    {
        return m_vertices[idx];
    }

    const Position & Polygon::operator()(const size_t idx) const
    {
        return m_vertices[idx];
    }

    Polygon Polygon::XYRectangleFromOffsets(const double front, const double back, const double left, const double right)
    {
        Polygon ret;
        ret.m_vertices.push_back(iDynTree::Position(front,left,0.0));
        ret.m_vertices.push_back(iDynTree::Position(-back,left,0.0));
        ret.m_vertices.push_back(iDynTree::Position(-back,-right,0.0));
        ret.m_vertices.push_back(iDynTree::Position(front,-right,0.0));

        return ret;
    }


    Polygon2D::Polygon2D(): m_vertices(0)
    {

    }

    bool Polygon2D::isValid() const
    {
        return (m_vertices.size() >= 3);
    }

    size_t Polygon2D::getNrOfVertices() const
    {
        return m_vertices.size();
    }

    void Polygon2D::setNrOfVertices(size_t size)
    {
        m_vertices.resize(size);
        return;
    }

    Vector2& Polygon2D::operator()(const size_t idx)
    {
        return m_vertices[idx];
    }

    const Vector2& Polygon2D::operator()(const size_t idx) const
    {
        return m_vertices[idx];
    }

    double monotono_chain_cross(Vector2 o, Vector2 a, Vector2 b)
    {
        return ((a(0)-o(0))*(b(1)-o(1))-(a(1)-o(1))*(b(0)-o(0)));
    }

    void ConvexHullProjectionConstraint::setActive(const bool isActive)
    {
        m_isActive = isActive;
    }

    bool ConvexHullProjectionConstraint::isActive()
    {
        return m_isActive;
    }

    size_t ConvexHullProjectionConstraint::getNrOfConstraints()
    {
        return A.rows();
    }

    bool ConvexHullProjectionConstraint::buildConvexHull(const iDynTree::Direction xAxisOfPlaneInWorld,
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

        // Compute the convex hull using the Monotone Chain

        // Order the projected points with a lexographical ordering
        struct
        {
            bool operator()(Vector2 a, Vector2 b)
            {
                return (a(0) < b(0)) || ( (a(0) == b(0)) && (a(1) < b(1)) );
            }
        } vector2lexographical;

        std::sort(projectedPoints.begin(), projectedPoints.end(), vector2lexographical);

        // Reset the convex hull (it will be properly resized after running the algorithm
        projectedConvexHull.m_vertices.resize(2*projectedPoints.size());
        size_t k = 0;

        // Build the upper hull
        for (int i = 0; i < projectedPoints.size(); ++i)
        {
            while (k >= 2 && monotono_chain_cross(projectedConvexHull.m_vertices[k-2],
                                                  projectedConvexHull.m_vertices[k-1],
                                                  projectedPoints[i]) <= 0)
            {
                k = k-1;
            }

            projectedConvexHull.m_vertices[k] = projectedPoints[i];
            k = k+1;

        }

        // Build the lower hull
        for (int i = projectedPoints.size()-2, t = k+1; i >= 0; i--)
        {
            while (k >= t && monotono_chain_cross(projectedConvexHull.m_vertices[k-2],
                                                  projectedConvexHull.m_vertices[k-1],
                                                  projectedPoints[i]) <= 0)
            {
                k = k-1;
            }

            projectedConvexHull.m_vertices[k] = projectedPoints[i];
            k = k+1;
        }

        projectedConvexHull.m_vertices.resize(k-1);

        // Build the constraint matrix
        buildConstraintMatrix();

        // Update AtimesP matrix
        AtimesP.resize(A.rows(),P.cols());

        toEigen(AtimesP) = toEigen(A)*toEigen(P);

        return true;
    }

    void ConvexHullProjectionConstraint::buildConstraintMatrix()
    {
        // The rows of the A matrix and of the b vector depends on the number of vertices in the convex hull
        A.resize(projectedConvexHull.getNrOfVertices(),2);
        b.resize(projectedConvexHull.getNrOfVertices());

        // We assume that the vertices in projectedConvexHull are expressed in counter-clockwise order: this
        // is ensured by the structure of the Monotone Chain algorithm

        // Iterate on all line segments one by one
        for (size_t i=0; i < projectedConvexHull.getNrOfVertices(); i++)
        {
            Vector2 p0 = projectedConvexHull(i);
            Vector2 p1;

            if ( i != projectedConvexHull.getNrOfVertices()-1 )
            {
                p1 = projectedConvexHull(i+1);
            }
            else
            {
                // if we are using the last vertix, we build the line connecting it with the first vertex
                p1 = projectedConvexHull(0);
            }

            toEigen(A).block<1,2>(i,0) = Eigen::Vector2d(p1(1)-p0(1),p0(0)-p1(0));
            b(i) = p0(0)*p1(1) - p1(0)*p0(1);
        }

        return;
    }

    Vector2 ConvexHullProjectionConstraint::project(Position& pos3dInAbsoluteFrame)
    {
        iDynTree::Vector2 projected;
        toEigen(projected) = toEigen(P) * toEigen(pos3dInAbsoluteFrame - o);
        return projected;
    }

    double distanceBetweentTwoPoints(const Vector2& firstPoint,
                                     const Vector2& secondPoint)
    {
        return (toEigen(firstPoint)-toEigen(secondPoint)).norm();
    }

    double distanceBetweenPointAndSegment(const Vector2& point,
                                          const Vector2& segmentFirstPoint,
                                          const Vector2& segmentLastPoint)
    {
        double segmentLengthSquared = (toEigen(segmentLastPoint)-toEigen(segmentFirstPoint)).squaredNorm();

        // First case: the segment is actually a point
        if (segmentLengthSquared == 0.0)
        {
            return distanceBetweentTwoPoints(point, segmentFirstPoint);
        }

        // The segment can be written as the set of segment points sp that can be written as:
        // sp = segmentFirstPoint + t*( segmentLastPoint - segmentFirstPoint )
        // for t \in [0, 1]
        // To find the segment point closest to the point, we can just find the t of the projection
        // of the point in the segment, and clamp it to [0, 1]
        double tProjection = ((toEigen(point)-toEigen(segmentFirstPoint)).dot(toEigen(segmentLastPoint)-toEigen(segmentFirstPoint)))/segmentLengthSquared;

        double tClosestPoint = std::max(0.0, std::min(1.0, tProjection));

        Vector2 closestPoint;
        toEigen(closestPoint) = toEigen(segmentFirstPoint) + tClosestPoint*(toEigen(segmentLastPoint)-toEigen(segmentFirstPoint));

        return distanceBetweentTwoPoints(point, closestPoint);
    }

    double ConvexHullProjectionConstraint::computeMargin(const Vector2& posIn2D)
    {
        bool isInside = true;

        // First check if the point is inside the convex hull or not
        iDynTree::VectorDynSize Ax(A.rows());

        toEigen(Ax) = toEigen(A)*toEigen(posIn2D);

        // If even one of the constraint is violated, then the point is outside the convex hull
        for (int i=0; i < Ax.size(); i++)
        {
            if (!(Ax(i) <= b(i)))
            {
                isInside = false;
                break;
            }
        }

        // Then, compute the distance between each segment of the convex hull and the point :
        // the minimum one is the distance of the point from the convex hull

        // We start from the last segment that do not follow the segment[i],segment[i+1] structure
        double distanceWithoutSign = distanceBetweenPointAndSegment(posIn2D,
                                                                    projectedConvexHull(projectedConvexHull.getNrOfVertices()-1),
                                                                    projectedConvexHull(0));
        // Find the minimum distance
        for (int i=0; i < projectedConvexHull.getNrOfVertices()-1; i++)
        {
            double candidateDistance = distanceBetweenPointAndSegment(posIn2D,
                                                                      projectedConvexHull(i),
                                                                      projectedConvexHull(i+1));

            if (candidateDistance < distanceWithoutSign)
            {
                distanceWithoutSign = candidateDistance;
            }
        }

        double margin;
        // If the point is inside the convex hull, we return the distance, otherwise the negated distance
        if (isInside)
        {
            margin = distanceWithoutSign;
        }
        else
        {
            margin = -distanceWithoutSign;
        }

        return margin;
    }

    void ConvexHullProjectionConstraint::setProjectionAlongDirection(Vector3 direction)
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

    Vector2 ConvexHullProjectionConstraint::projectAlongDirection(iDynTree::Position& posIn3dInAbsoluteFrame)
    {
        iDynTree::Vector2 projected;
        toEigen(projected) = toEigen(Pdirection) * toEigen(posIn3dInAbsoluteFrame - o);
        return projected;
    }

}
