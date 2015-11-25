/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Core/AngularMotionVector3.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/RotationalInertiaRaw.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/Utils.h>

#include <Eigen/Dense>

#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{
    /**
     * Local static functions
     */

    template <class SpatialMotionForceVectorT>
    static SpatialMotionForceVectorT changeCoordFrameOfT(const Rotation & rot,
                                                         const SpatialMotionForceVectorT & other)
    {
        return SpatialMotionForceVectorT(other.getLinearVec3().changeCoordFrame(rot),
                                         other.getAngularVec3().changeCoordFrame(rot));
    }

    /**
     * class Method definitions
     */


    typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;


    Rotation::Rotation(): RotationRaw()
    {
    }

    Rotation::Rotation(double xx, double xy, double xz,
                       double yx, double yy, double yz,
                       double zx, double zy, double zz): RotationRaw(xx,xy,xz,
                                                                     yx,yy,yz,
                                                                     zx,zy,zz)
    {
    }

    Rotation::Rotation(const Rotation & other): RotationRaw(other)
    {
        this->semantics = other.getSemantics();
    }

    Rotation::Rotation(const RotationRaw& other): RotationRaw(other)
    {

    }

    Rotation::Rotation(const RotationRaw& otherPos, RotationSemantics & otherSem): RotationRaw(otherPos)
    {
        this->semantics = otherSem;
    }

    RotationSemantics& Rotation::getSemantics()
    {
        return this->semantics;
    }

    const RotationSemantics& Rotation::getSemantics() const
    {
        return this->semantics;
    }

    const Rotation& Rotation::changeOrientFrame(const Rotation& newOrientFrame)
    {
        iDynTreeAssert( this->semantics.changeOrientFrame(newOrientFrame.semantics) );
        this->RotationRaw::changeOrientFrame(newOrientFrame);
        return *this;
    }

    const Rotation& Rotation::changeRefOrientFrame(const Rotation& newRefOrientFrame)
    {
        iDynTreeAssert( this->semantics.changeRefOrientFrame(newRefOrientFrame.semantics) );
        this->RotationRaw::changeRefOrientFrame(newRefOrientFrame);
        return *this;
    }

    const Rotation& Rotation::changeCoordinateFrame(const Rotation& newCoordinateFrame)
    {
        return this->changeRefOrientFrame(newCoordinateFrame);
    }

    Rotation Rotation::compose(const Rotation& op1, const Rotation& op2)
    {
        RotationSemantics resultSemantics;
        iDynTreeAssert( RotationSemantics::compose(op1.semantics,op2.semantics,resultSemantics) );
        return Rotation(RotationRaw::compose(op1,op2),resultSemantics);
    }

    Rotation Rotation::inverse2(const Rotation& orient)
    {
        RotationSemantics resultSemantics;
        iDynTreeAssert( RotationSemantics::inverse2(orient.getSemantics(),resultSemantics) );
        return Rotation(RotationRaw::inverse2(orient),resultSemantics);
    }

    Position Rotation::changeCoordFrameOf(const Position & other) const
    {
        PositionSemantics resultSemantics;
        iDynTreeAssert( this->semantics.changeCoordFrameOf(other.getSemantics(), resultSemantics) );
        return Position(this->RotationRaw::changeCoordFrameOf(other), resultSemantics);
    }

    SpatialMotionVector Rotation::changeCoordFrameOf(const SpatialMotionVector& other) const
    {
        return changeCoordFrameOfT<SpatialMotionVector>(*this, other);
    }

    SpatialForceVector Rotation::changeCoordFrameOf(const SpatialForceVector& other) const
    {
        return changeCoordFrameOfT<SpatialForceVector>(*this, other);
    }

    Twist Rotation::changeCoordFrameOf(const Twist& other) const
    {
        return changeCoordFrameOfT<Twist>(*this, other);
    }

    SpatialAcc Rotation::changeCoordFrameOf(const SpatialAcc & other) const
    {
        return changeCoordFrameOfT<SpatialAcc>(*this, other);
    }

    SpatialMomentum Rotation::changeCoordFrameOf(const SpatialMomentum & other) const
    {
        return changeCoordFrameOfT<SpatialMomentum>(*this, other);
    }

    Wrench Rotation::changeCoordFrameOf(const Wrench &other) const
    {
        return changeCoordFrameOfT<Wrench>(*this, other);
    }

    Direction Rotation::changeCoordFrameOf(const Direction& other) const
    {
        Direction result;

        // \todo TODO add semantics to Direction
        Eigen::Map<const Matrix3dRowMajor> newCoordFrame(m_data);
        Eigen::Map<const Eigen::Vector3d> directionCoord(other.data());
        Eigen::Map<Eigen::Vector3d> resultData(result.data());

        resultData = newCoordFrame*directionCoord;

        return result;
    }

    ClassicalAcc Rotation::changeCoordFrameOf(const ClassicalAcc &other) const
    {
        ClassicalAcc result;

        // \todo TODO add semantics to ClassicalAcc
        result = RotationRaw::changeCoordFrameOf(other);

        return result;
    }

    RotationalInertiaRaw Rotation::changeCoordFrameOf(const RotationalInertiaRaw &other) const
    {
        RotationalInertiaRaw result;

        // \todo TODO add semantics to ClassicalAcc
        result = RotationRaw::changeCoordFrameOf(other);

        return result;
    }

    Axis Rotation::changeCoordFrameOf(const Axis& other) const
    {
        return Axis(this->changeCoordFrameOf(other.getDirection()),this->changeCoordFrameOf(other.getOrigin()));
    }

    Rotation Rotation::inverse() const
    {
        return inverse2(*this);
    }

    Rotation Rotation::operator*(const Rotation& other) const
    {
        return compose(*this,other);
    }

    Position Rotation::operator*(const Position& other) const
    {
        return changeCoordFrameOf(other);
    }

    SpatialForceVector Rotation::operator*(const SpatialForceVector& other) const
    {
        return changeCoordFrameOfT<SpatialForceVector>(*this, other);
    }

    Twist Rotation::operator*(const Twist& other) const
    {
        return changeCoordFrameOfT<Twist>(*this, other);
    }

    SpatialAcc Rotation::operator*(const SpatialAcc & other) const
    {
        return changeCoordFrameOfT<SpatialAcc>(*this, other);
    }

    SpatialMomentum Rotation::operator*(const SpatialMomentum & other) const
    {
        return changeCoordFrameOfT<SpatialMomentum>(*this, other);
    }

    Wrench Rotation::operator*(const Wrench& other) const
    {
        return changeCoordFrameOfT<Wrench>(*this, other);
    }

    Direction Rotation::operator*(const Direction& other) const
    {
        return changeCoordFrameOf(other);
    }

    ClassicalAcc Rotation::operator*(const ClassicalAcc& other) const
    {
        return changeCoordFrameOf(other);
    }

    Axis Rotation::operator*(const Axis& other) const
    {
        return changeCoordFrameOf(other);
    }

    RotationalInertiaRaw Rotation::operator*(const RotationalInertiaRaw& other) const
    {
        return changeCoordFrameOf(other);
    }

    void Rotation::getRPY(double& r, double& p, double& y)
    {
        Eigen::Map<const Matrix3dRowMajor> dataEigen(this->data());

        Eigen::Vector3d rpy = dataEigen.eulerAngles(0,1,2);

        r = rpy(0);
        p = rpy(1);
        y = rpy(2);
    }


    AngularMotionVector3 Rotation::log() const
    {
        AngularMotionVector3 ret;

        Eigen::AngleAxisd aa(Eigen::Map<const Matrix3dRowMajor>(this->data()));

        // Implementation inspired from DART, see
        // https://github.com/dartsim/dart/pull/407/files
        // https://github.com/dartsim/dart/pull/334
        // https://github.com/dartsim/dart/issues/88
        std::cout << aa.angle() << std::endl;
        std::cout << aa.axis()  << std::endl;
        Eigen::Map<Eigen::Vector3d>(ret.data()) = aa.angle()*aa.axis();

        return ret;
    }

    Rotation Rotation::RotX(const double angle)
    {
        return Rotation(RotationRaw::RotX(angle));
    }

    Rotation Rotation::RotY(const double angle)
    {
        return Rotation(RotationRaw::RotY(angle));
    }

    Rotation Rotation::RotZ(const double angle)
    {
        return Rotation(RotationRaw::RotZ(angle));
    }

    Rotation Rotation::RotAxis(const Direction & direction, const double angle)
    {
        Rotation result;
        Eigen::Map<Matrix3dRowMajor> thisData(result.data());
        Eigen::Map<const Eigen::Vector3d>   directionData(direction.data());
        thisData = Eigen::AngleAxisd(angle, directionData).matrix();

        return result;
    }


    Rotation Rotation::RPY(const double roll, const double pitch, const double yaw)
    {
        return Rotation(RotationRaw::RPY(roll, pitch, yaw));
    }

    Rotation Rotation::Identity()
    {
        return RotationRaw::Identity();
    }

    std::string Rotation::toString() const
    {
        std::stringstream ss;

        ss << RotationRaw::toString() << " " << semantics.toString();

        return ss.str();
    }

    std::string Rotation::reservedToString() const
    {
        return this->toString();
    }


}
