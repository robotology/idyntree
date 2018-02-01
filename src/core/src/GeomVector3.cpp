/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/GeomVector3.h>
#include <iDynTree/Core/LinearMotionVector3.h>
#include <iDynTree/Core/AngularMotionVector3.h>
#include <iDynTree/Core/LinearForceVector3.h>
#include <iDynTree/Core/AngularForceVector3.h>
#include <iDynTree/Core/PrivatePreProcessorUtils.h>
#include <iDynTree/Core/PrivateSemanticsMacros.h>


#include <Eigen/Dense>

#define GEOMVECTOR3SEMANTICS_INSTANCE_HDR \
GeomVector3Semantics<MotionForceSemanticsT>

#define GEOMVECTOR3_INSTANCE_HDR \
GeomVector3<MotionForceT>

namespace iDynTree
{
    typedef Eigen::Matrix<double,3,1> Vector3d;
    typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

    /**
     * GeomVector3Semantics Method definitions ===================================
     */

    // constructors
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    GEOMVECTOR3SEMANTICS_INSTANCE_HDR::GeomVector3Semantics(int _body, int _refBody, int _coordinateFrame):
    body(_body),
    refBody(_refBody),
    coordinateFrame(_coordinateFrame)
    {
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    GEOMVECTOR3SEMANTICS_INSTANCE_HDR::GeomVector3Semantics(const GeomVector3Semantics & other):
    body(other.body),
    refBody(other.refBody),
    coordinateFrame(other.coordinateFrame)
    {
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    void GEOMVECTOR3SEMANTICS_INSTANCE_HDR::setToUnknown()
    {
        body = UNKNOWN;
        refBody = UNKNOWN;
        coordinateFrame = UNKNOWN;
    }

    // Getters, setters & helpers
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    int GEOMVECTOR3SEMANTICS_INSTANCE_HDR::getBody() const
    {
        return this->body;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    int GEOMVECTOR3SEMANTICS_INSTANCE_HDR::getRefBody() const
    {
        return this->refBody;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    int GEOMVECTOR3SEMANTICS_INSTANCE_HDR::getCoordinateFrame() const
    {
        return this->coordinateFrame;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::isUnknown() const
    {
        return (this->body == UNKNOWN && this->refBody == UNKNOWN && this->coordinateFrame == UNKNOWN);
    }

    // Semantics operations
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::changeCoordFrame(const RotationSemantics & newCoordFrame,
                                                             MotionForceSemanticsT & result) const
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(newCoordFrame.getOrientationFrame(),this->coordinateFrame),
                          IDYNTREE_PRETTY_FUNCTION,
                          "newCoordFrame orientationFrame is different from current Motion/Force vector's coordinateFrame\n")/*
                                                                                                                              && reportErrorIf(!checkEqualOrUnknown(newCoordFrame.getBody(),this->refBody),
                                                                                                                              IDYNTREE_PRETTY_FUNCTION,
                                                                                                                              "newCoordFrame body is different from current Motion/Force vector's reference body\n")
                                                                                                                              && reportErrorIf(!checkEqualOrUnknown(newCoordFrame.getBody(),newCoordFrame.getRefBody()),
                                                                                                                              IDYNTREE_PRETTY_FUNCTION,
                                                                                                                              "newCoordFrame body and reference body should be the same\n")*/);

        // compute semantics
        //result = *this; won't compile because "this" is compiled as a "GeomVector3Semantics" object.
        result.body = this->body;
        result.refBody = this->refBody;
        result.coordinateFrame = this->coordinateFrame;

        result.refBody = newCoordFrame.getRefBody();
        result.coordinateFrame = newCoordFrame.getCoordinateFrame();

        return semantics_status;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::compose(const MotionForceSemanticsT & op1, const MotionForceSemanticsT & op2, MotionForceSemanticsT & result)
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame),
                          IDYNTREE_PRETTY_FUNCTION,
                          "composing two geometric relations expressed in different coordinateFrames\n")
         && reportErrorIf(!checkEqualOrUnknown(op1.refBody,op2.body) && (op1.body == op2.refBody),
                          IDYNTREE_PRETTY_FUNCTION,
                          "angVelocity(A,C) = compose(angVelocity(B,C),angVelocity(A,B)) is forbidded in iDynTree to avoid ambiguity on compose(angVelocity(B,A),angVelocity(A,B)). Same for other motion or force 3D vectors.\n")
         && reportErrorIf(!checkEqualOrUnknown(op1.refBody,op2.body),
                          IDYNTREE_PRETTY_FUNCTION,
                          "op1 reference body and op2 body don't match\n"));

        // compute semantics;
        result = op1;
        result.refBody = op2.refBody;

        return semantics_status;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::inverse(const MotionForceSemanticsT & op, MotionForceSemanticsT & result)
    {
        // compute semantics
        result = op;
        result.body = op.refBody;
        result.refBody = op.body;

        return true;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::dot(const typename DualMotionForceSemanticsT<MotionForceSemanticsT>::Type & other) const
    {
        // check semantics
        bool semantics_status =
        (   reportErrorIf(!checkEqualOrUnknown(this->coordinateFrame,other.getCoordinateFrame()),
                          IDYNTREE_PRETTY_FUNCTION,
                          "multiplying two geometric relations expressed in different coordinateFrames\n")
         && reportErrorIf(!checkEqualOrUnknown(this->body,other.getBody()),
                          IDYNTREE_PRETTY_FUNCTION,
                          "The bodies defined for both operands of the dot product don't match\n")
         && reportErrorIf(!checkEqualOrUnknown(this->refBody,other.getRefBody()),
                          IDYNTREE_PRETTY_FUNCTION,
                          "The reference bodies defined for both operands of the dot product don't match\n"));

        // compute semantics: the result of dot product is a scalar and we're not sure if this result's
        // semantics make sense. For now it's not defined.

        return semantics_status;
    }


    /**
     * GeomVector3 Method definitions ============================================
     */

    // constructors
    GEOMVECTOR3_TEMPLATE_HDR
    GEOMVECTOR3_INSTANCE_HDR::GeomVector3(const double* in_data, const unsigned int in_size): Vector3(in_data, in_size)
    {}

    GEOMVECTOR3_TEMPLATE_HDR
    GEOMVECTOR3_INSTANCE_HDR::GeomVector3(const GeomVector3 & other): Vector3(other)
    {
        iDynTreeSemanticsOp(semantics = other.semantics);
    }

    // Getters & setters
    GEOMVECTOR3_TEMPLATE_HDR
    const typename GEOMVECTOR3_INSTANCE_HDR::MotionForceSemanticsT& GEOMVECTOR3_INSTANCE_HDR::getSemantics() const
    {
        return semantics;
    }

    GEOMVECTOR3_TEMPLATE_HDR
    void GEOMVECTOR3_INSTANCE_HDR::setSemantics(MotionForceSemanticsT& _semantics)
    {
        iDynTreeAssert(this->semantics.isUnknown());
        this->semantics = _semantics;
    }

    // Geometric operations
    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::changeCoordFrame(const Rotation & newCoordFrame) const
    {
        MotionForceT result;

        iDynTreeAssert(semantics.changeCoordFrame(newCoordFrame.getSemantics(), result.semantics));

        Eigen::Map<const Vector3d> thisMap(this->data());
        Eigen::Map<const Matrix3dRowMajor> rotMap(newCoordFrame.data());
        Eigen::Map<Vector3d> resultMap(result.data());

        resultMap = rotMap*thisMap;

        return result;
    }

    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::compose(const MotionForceTbase & op1, const MotionForceT & op2)
    {
        MotionForceT result;

        iDynTreeAssert(MotionForceSemanticsT::compose(op1.semantics, op2.semantics, result.semantics));

        Eigen::Map<const Vector3d> op1Data(op1.data());
        Eigen::Map<const Vector3d> op2Data(op2.data());
        Eigen::Map<Vector3d> resultData(result.data());

        resultData = op1Data + op2Data;

        return result;
    }

    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::inverse(const MotionForceTbase & op)
    {
        MotionForceT result;

        iDynTreeAssert(GeomVector3Semantics<MotionForceSemanticsT>::inverse(op.semantics, result.semantics));

        Eigen::Map<const Vector3d> opData(op.data());
        Eigen::Map<Vector3d> resultData(result.data());

        resultData = -opData;

        return result;
    }

    // dot product
    GEOMVECTOR3_TEMPLATE_HDR
    double GEOMVECTOR3_INSTANCE_HDR::dot(const typename MotionForce_traits<MotionForceT>::DualSpace & other) const
    {
        iDynTreeAssert(semantics.dot(other.semantics));

        Eigen::Map<const Vector3d> otherData(other.data());
        Eigen::Map<const Vector3d> thisData(this->data());

        return thisData.dot(otherData);
    }

    // overloaded operators
    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::operator+(const MotionForceT &other) const
    {
        return compose(*this, other);
    }

    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::operator-(const MotionForceT &other) const
    {
        return compose(*this, inverse(other));
    }

    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::operator-() const
    {
        return inverse(*this);
    }

    /**
     * Instantiations for avoiding linker issues
     */
    template class GeomVector3Semantics<LinearMotionVector3Semantics>;
    template class GeomVector3Semantics<AngularMotionVector3Semantics>;
    template class GeomVector3Semantics<LinearForceVector3Semantics>;
    template class GeomVector3Semantics<AngularForceVector3Semantics>;

    template class GeomVector3<LinearMotionVector3>;
    template class GeomVector3<AngularMotionVector3>;
    template class GeomVector3<LinearForceVector3>;
    template class GeomVector3<AngularForceVector3>;

}
