/*!
 * @file ConvexHullHelpers.cpp
 * @author Francesco Romano
 * @copyright 2017 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2017
 */

#include <iDynTree/ConvexHullHelpers.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <Eigen/Core>

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
        toEigen(P).block<1,3>(0,0) = toEigen(xAxisOfPlaneInWorld);
        toEigen(P).block<1,3>(1,0) = toEigen(yAxisOfPlaneInWorld);

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
                 Vector2 newProjectedPoint;
                 toEigen(newProjectedPoint) = toEigen(P)*toEigen(supportPolygonsExpressedInAbsoluteFrame[poly](vertex)-o);
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
}