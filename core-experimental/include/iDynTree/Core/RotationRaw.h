/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ROTATION_RAW_H
#define IDYNTREE_ROTATION_RAW_H

#include "IMatrix.h"
#include <string>

namespace iDynTree
{
    class PositionRaw;
    class SpatialMotionVectorRaw;
    class SpatialForceVectorRaw;

    /**
     * Class providing the raw coordinates for iDynTree::Rotation class.
     *
     * \ingroup iDynTreeCore
     */
    class RotationRaw: public IMatrix
    {
    protected:
        /**
         * Storage for the Orientation:
         * The rotation matrix representation of the orientation, stored in row major order.
         *
         * \note This implementation is compatible with KDL::Rotation data.
         */
        double privateData[9];


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
         * Copy constructor: create a RotationRaw from another RotationRaw.
         */
        RotationRaw(const RotationRaw & other);

        /**
         * Denstructor
         */
        virtual ~RotationRaw();

        /**
         * @name Matrix interface methods.
         * Methods exposing a vector-like interface to RotationRaw.
         *
         * \warning Notice that using this methods you can damage the underlyng rotation matrix.
         *          In doubt, don't use them and rely on more high level functions.
         */
        ///@{
        double operator()(const unsigned int row, const unsigned int col) const;
        double& operator()(const unsigned int row, const unsigned int col);
        double getVal(const unsigned int row, const unsigned int col) const;
        bool setVal(const unsigned int row, const unsigned int col, const double new_el);
        unsigned int rows() const;
        unsigned int cols() const;
        ///@}


        /**
         * Raw data accessor: return a const pointer to a 9 element buffer,
         * where the rotation matrix is stored following the row major order.
         */
        const double * data() const;

        /**
         * Raw data accessor: return a pointer to a 9 element buffer,
         * where the rotation matrix is stored following the row major order.
         */
        double * data();

        const RotationRaw & changeOrientFrame(const RotationRaw & newOrientFrame);
        const RotationRaw & changeRefOrientFrame(const RotationRaw & newRefOrientFrame);
        static RotationRaw compose(const RotationRaw & op1, const RotationRaw & op2);
        static RotationRaw inverse2(const RotationRaw & orient);
        static PositionRaw transform(const RotationRaw & op1, const PositionRaw & op2);
        static SpatialMotionVectorRaw transform(const RotationRaw & op1, const SpatialMotionVectorRaw & op2);
        static SpatialForceVectorRaw transform(const RotationRaw & op1,  const SpatialForceVectorRaw & op2);


        /** overloaded operators **/
        RotationRaw operator*(const RotationRaw & other) const;
        RotationRaw inverse() const;
        PositionRaw operator*(const PositionRaw & other) const;
        SpatialMotionVectorRaw operator*(const SpatialMotionVectorRaw & other) const;
        SpatialForceVectorRaw operator*(const SpatialForceVectorRaw & other) const;


        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

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
    };
}

#endif