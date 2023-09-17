// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Position.h>
#include <iDynTree/Rotation.h>
#include <iDynTree/Twist.h>
#include <iDynTree/Wrench.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/Utils.h>

#include <iDynTree/EigenHelpers.h>

#include <cassert>
#include <iostream>
#include <sstream>


namespace iDynTree
{
    /**
     * Local static functions
     */

    template <class SpatialMotionForceVectorT>
    static SpatialMotionForceVectorT changePointOfMotionT(const Position & pos,
                                                          const SpatialMotionForceVectorT & other)
    {
        GeomVector3 newLinearVec;
        toEigen(newLinearVec) = toEigen(other.getLinearVec3()) + toEigen(pos).cross(toEigen(other.getAngularVec3()));
        return SpatialMotionForceVectorT(newLinearVec,
                                         other.getAngularVec3());
    }

    template <class SpatialMotionForceVectorT>
    static SpatialMotionForceVectorT changePointOfForceT(const Position & pos,
                                                         const SpatialMotionForceVectorT & other)
    {
        GeomVector3 newAngularVec;
        toEigen(newAngularVec) = toEigen(other.getAngularVec3()) + toEigen(pos).cross(toEigen(other.getLinearVec3()));
        return SpatialMotionForceVectorT(other.getLinearVec3(),
                                         newAngularVec);
    }

    Position::Position()
    {
    }


    Position::Position(double x, double y, double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;
    }

    Position::Position(const double* in_data, const unsigned int in_size):
                 VectorFixSize< 3 >(in_data,in_size)
    {

    }

    Position::Position(const Position& other):VectorFixSize< int(3) >(other)
    {
        this->m_data[0] = other.m_data[0];
        this->m_data[1] = other.m_data[1];
        this->m_data[2] = other.m_data[2];

    }

    Position& Position::operator=(const Position& other)
    {
        this->m_data[0] = other.m_data[0];
        this->m_data[1] = other.m_data[1];
        this->m_data[2] = other.m_data[2];
        return *this;
    }

    Position::Position(Span<const double> other):
                 VectorFixSize< 3 >(other)
    {

    }

    const Position& Position::changePoint(const Position& newPoint)
    {
        this->m_data[0] += newPoint(0);
        this->m_data[1] += newPoint(1);
        this->m_data[2] += newPoint(2);

        return *this;
    }

    const Position& Position::changeRefPoint(const Position& newRefPoint)
    {
        this->m_data[0] += newRefPoint(0);
        this->m_data[1] += newRefPoint(1);
        this->m_data[2] += newRefPoint(2);

        return *this;
    }

    Position Position::compose(const Position& op1, const Position& op2)
    {
        Position result;
        result(0) = op1(0) + op2(0);
        result(1) = op1(1) + op2(1);
        result(2) = op1(2) + op2(2);
        return result;
    }

    Position Position::inverse(const Position& op)
    {
        Position result;
        result(0) = -op.m_data[0];
        result(1) = -op.m_data[1];
        result(2) = -op.m_data[2];
        return result;
    }

    SpatialMotionVector Position::changePointOf(const SpatialMotionVector & other) const
    {
        SpatialMotionVector result;

        Eigen::Map<const Eigen::Vector3d> thisPos(this->data());
        Eigen::Map<const Eigen::Vector3d> otherLinear(other.getLinearVec3().data());
        Eigen::Map<const Eigen::Vector3d> otherAngular(other.getAngularVec3().data());
        Eigen::Map<Eigen::Vector3d> resLinear(result.getLinearVec3().data());
        Eigen::Map<Eigen::Vector3d> resAngular(result.getAngularVec3().data());

        resLinear  = otherLinear + thisPos.cross(otherAngular);
        resAngular = otherAngular;

        return result;
    }

    SpatialForceVector Position::changePointOf(const SpatialForceVector & other) const
    {
        SpatialForceVector result;

        Eigen::Map<const Eigen::Vector3d> thisPos(this->data());
        Eigen::Map<const Eigen::Vector3d> otherLinear(other.getLinearVec3().data());
        Eigen::Map<const Eigen::Vector3d> otherAngular(other.getAngularVec3().data());
        Eigen::Map<Eigen::Vector3d> resLinear(result.getLinearVec3().data());
        Eigen::Map<Eigen::Vector3d> resAngular(result.getAngularVec3().data());

        resLinear  = otherLinear;
        resAngular = thisPos.cross(otherLinear) + otherAngular;

        return result;
    }

    std::string Position::toString() const
    {
        std::stringstream ss;
        ss << this->m_data[0] << " " << this->m_data[1] << " " << this->m_data[2];
        return ss.str();
    }

    std::string Position::reservedToString() const
    {
        return this->toString();
    }

    const Position& Position::changeCoordinateFrame(const Rotation & newCoordinateFrame)
    {
        *this = newCoordinateFrame.changeCoordFrameOf(*this);
        return *this;
    }


    Twist Position::changePointOf(const Twist & other) const
    {
        return changePointOfMotionT<Twist>(*this, other);
    }

    SpatialAcc Position::changePointOf(const SpatialAcc & other) const
    {
        return changePointOfMotionT<SpatialAcc>(*this, other);
    }

    Wrench Position::changePointOf(const Wrench & other) const
    {
        return changePointOfForceT<Wrench>(*this, other);
    }

    SpatialMomentum Position::changePointOf(const SpatialMomentum & other) const
    {
        return changePointOfForceT<SpatialMomentum>(*this, other);
    }

    // overloaded operators
    Position Position::operator+(const Position& other) const
    {
        return compose(*this,other);
    }

    Position Position::operator-(const Position& other) const
    {
        return compose(*this,inverse(other));
    }

    Position Position::operator-() const
    {
        return inverse(*this);
    }

    Twist Position::operator*(const Twist& other) const
    {
        return changePointOfMotionT<Twist>(*this, other);
    }

    SpatialAcc Position::operator*(const SpatialAcc& other) const
    {
        return changePointOfMotionT<SpatialAcc>(*this, other);
    }

    SpatialForceVector Position::operator*(const SpatialForceVector& other) const
    {
        return changePointOfForceT<SpatialForceVector>(*this, other);
    }

    SpatialMomentum Position::operator*(const SpatialMomentum& other) const
    {
        return changePointOfForceT<SpatialMomentum>(*this, other);
    }

    Wrench Position::operator*(const Wrench& other) const
    {
        return changePointOfForceT<Wrench>(*this, other);
    }

    Position Position::Zero()
    {
        Position ret;
        ret.zero();
        return ret;
    }


}
