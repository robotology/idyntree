/*!
 * @file  BoundingBoxHelpers.h
 * @author Jorhabib Eljaik
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 *
 */


#ifndef IDYNTREE_BOUNDINGBOXHELPERS_H
#define IDYNTREE_BOUNDINGBOXHELPERS_H

#include <string>
#include <vector>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/ConvexHullHelpers.h>

namespace iDynTree
{
    /**
     * BoundingBoxProjectionConstraint helper.
     *
     */
    class BoundingBoxProjectionConstraint
    {
        /**
         * Once you compute the projected bounding box, build the A matrix and the b vector such that
         * Ax <= b iff the center of mass projection x is inside the bounding box.
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
         * bounding box expressed in the 2D project constraint plane.
         *
         * This is computed by the buildBoundingBox method.
         */
        Polygon2D projectedBoundingBox;

        /**
         * A constraint matrix, such that Ax <= b iff the com projection x is in the bounding box.
         */
        MatrixDynSize A;

        /**
         * b vector, such that Ax <= b iff the com projection x is in the bounding box.
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
         * Matrix obtained multiplying the matrix A for the matrix P.
         */
        MatrixDynSize AtimesP;

        /**
         * Plane offset o
         * Note that x = P*(c-o), where x is the projection and c is the 3d COM .
         */
        iDynTree::Position o;

        /**
         * Build the projected bounding box.
         *
         * @param projectionPlaneXaxisInAbsoluteFrame X direction of the projection axis, in the absolute frame.
         * @param projectionPlaneYaxisInAbsoluteFrame Y direction of the projection axis, in the absolute frame.
         * @param supportPolygonsExpressedInSupportFrame Vector of the support polygons, expressed in the support frames.
         * @param absoluteFrame_X_supportFrame Vector of the transform between each support frame and the absolute frame.
         * @return true if all went well, false otherwise.
         */
        bool buildBoundingBox(const iDynTree::Direction xAxisOfPlaneInWorld,
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
         * Project a point in the plane of the bounding box.
         * The point is expressed in the absolute frame of the constriant.
         */
        Vector2 project(iDynTree::Position& posIn3dInAbsoluteFrame);

        /*!
         * Set the projection matrix 'Pdirection' given a desired projection direction.
         *
         * \author Aiko Dinale (29/08/2017)
         *
         * @param direction     vector along which we want to project a point
         */
        void setProjectionAlongDirection(Vector3 direction);

        /*!
         * Project a point along a direction defined by the projection matrix 'Pdirection'
         *
         * \author Aiko Dinale (29/08/2017)
         *
         * @param posIn3dInAbsoluteFrame     a point we want to project
         */
        Vector2 projectAlongDirection(iDynTree::Position& posIn3dInAbsoluteFrame);

    };
}


#endif
