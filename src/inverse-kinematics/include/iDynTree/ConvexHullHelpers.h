// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#ifndef IDYNTREE_CONVEXHULLHELPERS_H
#define IDYNTREE_CONVEXHULLHELPERS_H

#include <string>
#include <vector>

#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Position.h>
#include <iDynTree/Transform.h>
#include <iDynTree/VectorDynSize.h>


namespace iDynTree
{
    /**
     * Class representing a 2D Polygon expressed in the 3D space.
     *
     * A poligon is a geomtric object consisting of a number of points (called vertices) and an equal number of line segments (called sides),
     *  namely a cyclically ordered set of points in a plane, with no three successive points collinear, together with the line segments
     *  joining consecutive pairs of the points. In other words, a polygon is closed broken line lying in a plane.
     */
    class Polygon
    {
    public:
        std::vector<Position> m_vertices;

        /**
         * Default constructor: build an invalid polygon without any vertex.
         */
        Polygon();

        /**
         * Set the number of vertices (the vertices can then be accessed with the operator()
         */
        void setNrOfVertices(size_t size);

        /**
         * Get the number of vertices in the Polygon
         * @return the number of vertices
         */
        size_t getNrOfVertices() const;

        /**
         * Check if a polygon is valid.
         *
         * The condition for the validity of the polygon are:
         * It has at least three points.
         *
         * @return true if is valid, false otherwise.
         */
        bool isValid() const;

        /**
         * Apply a transform on the polygon.
         * @return the transformed polygon.
         */
        Polygon applyTransform(const Transform & newFrame_X_oldFrame) const;

        Position& operator()(const size_t idx);
        const Position & operator()(const size_t idx) const;

        /**
         * Return the polygon of a rectangle on the XY plane given
         * the front (x positive), back (x negative), left (y positive) and right (y negative) offsets.
         *
         */
        static Polygon XYRectangleFromOffsets(const double front, const double back, const double left, const double right);
    };

    /**
     * Class representing a 2D Polygon expressed in the 2D space.
     *
     */
    class Polygon2D
    {
    public:
        std::vector<Vector2> m_vertices;

        /**
         * Default constructor: build an invalid polygon without any vertex.
         */
        Polygon2D();

        /**
         * Set the number of vertices (the vertices can then be accessed with the operator()
         */
        void setNrOfVertices(size_t size);

        /**
         * Get the number of vertices in the Polygon
         * @return the number of vertices
         */
        size_t getNrOfVertices() const;

        /**
         * Check if a polygon is valid.
         *
         * The condition for the validity of the polygon are:
         * It has at least three points.
         *
         * @return true if is valid, false otherwise.
         */
        bool isValid() const;

        Vector2& operator()(const size_t idx);
        const Vector2 & operator()(const size_t idx) const;
    };

    /**
     * ConvexHullProjectionConstraint helper.
     *
     */
    class ConvexHullProjectionConstraint
    {
        /**
         * Once you compute the projected convex hull, build the A matrix and the b vector such that
         * Ax <= b iff the center of mass projection x is inside the convex hull.
         */
        void buildConstraintMatrix();

        /**
         * Flag to specify if the constraint is active or not.
         */
        bool m_isActive;
    public:
        /**
         * Set if the constraint is active or not.
         */
        void setActive(const bool isActive);

        /**
         * Get if the constraint is active or not.
         * @return true if the constraint is active, false otherwise.
         */
        bool isActive();

        /**
         * Get the number of constraints (i.e. the number rows of the matrix A).
         * @return the number of constraints.
         */
        size_t getNrOfConstraints();

        /**
         * Convex hull expressed in the 2D project constraint plane.
         *
         * This is computed by the buildConvexHull method.
         */
        Polygon2D projectedConvexHull;

        /**
         * A constraint matrix, such that Ax <= b iff the com projection x is in the convex hull.
         */
        MatrixDynSize A;

        /**
         * b vector, such that Ax <= b iff the com projection x is in the convex hull.
         */
        VectorDynSize b;

        /**
         * Projection matrix P,
         * Note that x = P*(c-o), where x is the projection and c is the 3d COM .
         */
        Matrix2x3 P;

        /**
         * Projection matrix 'Pdirection' defined by a given direction.
         * The projection 'x' of a 3D point 'c' along a given vector is obtained as:
         * x = Pdirection*(c-o).
         */
        Matrix2x3 Pdirection;

        /**
         * Matrix obtained multiplyng the matrix A for the matrix P.
         */
        MatrixDynSize AtimesP;

        /**
         * Plane offset o
         * Note that x = P*(c-o), where x is the projection and c is the 3d COM .
         */
        iDynTree::Position o;

        /**
         * Build the projected convex hull.
         *
         * @param projectionPlaneXaxisInAbsoluteFrame X direction of the projection axis, in the absolute frame.
         * @param projectionPlaneYaxisInAbsoluteFrame Y direction of the projection axis, in the absolute frame.
         * @param supportPolygonsExpressedInSupportFrame Vector of the support polygons, expressed in the support frames.
         * @param absoluteFrame_X_supportFrame Vector of the transform between each support frame and the absolute frame.
         * @return true if all went well, false otherwise.
         */
        bool buildConvexHull(const iDynTree::Direction xAxisOfPlaneInWorld,
                             const iDynTree::Direction yAxisOfPlaneInWorld,
                             const iDynTree::Position originOfPlaneInWorld,
                             const std::vector<Polygon> & supportPolygonsExpressedInSupportFrame,
                             const std::vector<Transform> & absoluteFrame_X_supportFrame);

        /**
         * List of support frames.
         */
        std::vector<int> supportFrameIndices;

        /**
         * List of absolue_X_supportFrames
         */
        std::vector<Transform> absoluteFrame_X_supportFrame;

        /**
         * Project a point in the plane of the convex hull.
         * The point is expressed in the absolute frame of the constriant.
         */
        Vector2 project(iDynTree::Position& posIn3dInAbsoluteFrame);

        /**
         * Compute distance of a 2D point from the convex hull.
         * The distance is positive if the point is inside the convex hull,
         * zero if the point is on the boundary of the convex hull,
         * and negative if it is outside of the convex hull.
         */
        double computeMargin(const Vector2& posIn2D);

        /*!
         * Set the projection matrix 'Pdirection' given a desired projection direction.
         *
         * \author Aiko Dinale (29/08/2017)
         *
         * @param direction     vector along which we want to project a point
         */
        void setProjectionAlongDirection(const iDynTree::Vector3& direction);

        /*!
         * Project a point along a direction defined by the projection matrix 'Pdirection'
         *
         * \author Aiko Dinale (29/08/2017)
         *
         * @param posIn3dInAbsoluteFrame     a point we want to project
         */
        Vector2 projectAlongDirection(const iDynTree::Position& posIn3dInAbsoluteFrame);

    };
}


#endif
