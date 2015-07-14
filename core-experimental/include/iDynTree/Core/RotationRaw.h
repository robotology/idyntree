/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ROTATION_RAW_H
#define IDYNTREE_ROTATION_RAW_H

#include <iDynTree/Core/IMatrix.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <string>

namespace iDynTree
{
    class PositionRaw;
    class SpatialMotionVectorRaw;
    class SpatialForceVectorRaw;

    /**
     * Class providing the raw coordinates for iDynTree::Rotation class.
     *
     * Storage for the Orientation:
     * The rotation matrix representation of the orientation, stored in row major order,
     * inside a Matrix3x3 parent object.
     *
     * \note This implementation is compatible with KDL::Rotation data.
     *
     * \warning This class is exposing for convenience the IVector interface.
     *          Notice that using this methods you can damage the underlyng rotation matrix.
     *          In doubt, don't use them and rely on more high level functions.
     *
     * \ingroup iDynTreeCore
     */
    class RotationRaw: public Matrix3x3
    {
        public:
        /**
         * Default constructor: initialize all the rotation to the identity
         */
        RotationRaw();

        /**
         * Constructor from 9 doubles: initialize elements of the rotation matrix.
         */
        RotationRaw(double xx, double xy, double xz,
                    double yx, double yy, double yz,
                    double zx, double zy, double zz);

        /**
         * Constructor from a buffer of 9 doubles,
         * stored as a C-style array (i.e. row major).
         *
         */
        RotationRaw(const double* in_data,
                    const unsigned int in_rows,
                    const unsigned int in_cols);

        /**
         * Copy constructor: create a RotationRaw from another RotationRaw.
         */
        RotationRaw(const RotationRaw & other);

        /**
         * Denstructor
         */
        virtual ~RotationRaw();

        /**
         * Geometric operations.
         */
        const RotationRaw & changeOrientFrame(const RotationRaw & newOrientFrame);
        const RotationRaw & changeRefOrientFrame(const RotationRaw & newRefOrientFrame);
        static RotationRaw compose(const RotationRaw & op1, const RotationRaw & op2);
        static RotationRaw inverse2(const RotationRaw & orient);
        PositionRaw changeCoordFrameOf(const PositionRaw & other) const;
        SpatialMotionVectorRaw changeCoordFrameOf(const SpatialMotionVectorRaw & other) const;
        SpatialForceVectorRaw changeCoordFrameOf(const SpatialForceVectorRaw & other) const;


        /**
         * overloaded operators
         */

        /**
         * @name Initialization helpers.
         *
         */
        ///@{

        /**
         * Return a Rotation around axis X of given angle
         *
         * @param angle the angle (in Radians) of the rotation arount the X axis
         */
        static RotationRaw RotX(const double angle);

        /**
         * Return a Rotation around axis Y of given angle
         *
         * @param angle the angle (in Radians) of the rotation arount the Y axis
         */
        static RotationRaw RotY(const double angle);

        /**
         * Return a Rotation around axis Z of given angle
         *
         * @param angle the angle (in Radians) of the rotation arount the Z axis
         */
        static RotationRaw RotZ(const double angle);

        /**
         * Return a rotation object given Roll, Pitch and Yaw values.
         *
         * @note This method is compatible with the KDL::Rotation::RPY method.
         */
        static RotationRaw RPY(const double roll, const double pitch, const double yaw);

        /**
         * Return an identity rotation.
         *
         *
         */
        static RotationRaw Identity();

        ///@}

        
        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        
        std::string reservedToString() const;
        ///@}
    };
}

#endif