// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_DYNAMICS_LINEARIZATION_HELPERS_H
#define IDYNTREE_DYNAMICS_LINEARIZATION_HELPERS_H

#include <iDynTree/MatrixFixSize.h>

#include <iDynTree/Dynamics.h>

namespace iDynTree
{
    /**
     * Class representing the derivative
     * of a spatial motion vector with respect to
     * another spatial motion vector.
     */
    class SpatialMotionWrtMotionDerivative: public Matrix6x6
    {
    public:
        /**
         * Equivalent to:
         *
         * ```
         * SpatialMotionWrtMotionDerivative ret;
         * toEigen(ret) = toEigen(ret)*toEigen(transform.asAdjointMatrix())
         * return ret;
         * ```
         */
        SpatialMotionWrtMotionDerivative operator*(const Transform & a_X_b);

     };

     /**
       * Equivalent to:
       *
       * ```
       * SpatialForceWrtMotionDerivative ret;
       * toEigen(ret) = toEigen(transform.asAdjointMatrixWrench())*toEigen(ret);
       * return ret;
       * ```
       */
    SpatialMotionWrtMotionDerivative operator*(const Transform & a_X_b, const SpatialMotionWrtMotionDerivative & op2);


    /**
     * Class representing the derivative
     * of a spatial force vector with respect to
     * a spatial motion vector.
     */
    class SpatialForceWrtMotionDerivative: public Matrix6x6
    {
    public:
        /**
         * Equivalent to:
         *
         * ```
         * SpatialForceWrtMotionDerivative ret;
         * toEigen(ret) = toEigen(ret)*toEigen(transform.asAdjointMatrix())
         * return ret;
         * ```
         */
        SpatialForceWrtMotionDerivative operator*(const Transform & a_X_b);

    };


     /**
      * Equivalent to:
      *
      * ```
      * SpatialForceWrtMotionDerivative ret;
      * toEigen(ret) = toEigen(transform.asAdjointMatrixWrench())*toEigen(ret);
      * return ret;
      * ```
      */
     SpatialForceWrtMotionDerivative operator*(const Transform & a_X_b, const SpatialForceWrtMotionDerivative & op2);

}


#endif /* IDYNTREE_DYNAMICS_LINEARIZATION_HELPERS_H */