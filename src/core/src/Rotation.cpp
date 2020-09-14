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
#include <iDynTree/Core/EigenHelpers.h>


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
    }

    Rotation::Rotation(const RotationRaw& other): RotationRaw(other)
    {

    }

    Rotation::Rotation(MatrixView<const double> other): RotationRaw(other)
    {

    }

    const Rotation& Rotation::changeOrientFrame(const Rotation& newOrientFrame)
    {
        this->RotationRaw::changeOrientFrame(newOrientFrame);
        return *this;
    }

    const Rotation& Rotation::changeRefOrientFrame(const Rotation& newRefOrientFrame)
    {
        this->RotationRaw::changeRefOrientFrame(newRefOrientFrame);
        return *this;
    }

    const Rotation& Rotation::changeCoordinateFrame(const Rotation& newCoordinateFrame)
    {
        return this->changeRefOrientFrame(newCoordinateFrame);
    }

    Rotation Rotation::compose(const Rotation& op1, const Rotation& op2)
    {
        return Rotation(RotationRaw::compose(op1,op2));
    }

    Rotation Rotation::inverse2(const Rotation& orient)
    {
        return Rotation(RotationRaw::inverse2(orient));
    }

    Position Rotation::changeCoordFrameOf(const Position & other) const
    {
        return Position(this->RotationRaw::changeCoordFrameOf(other));
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

        Eigen::Map<const Matrix3dRowMajor> newCoordFrame(m_data);
        Eigen::Map<const Eigen::Vector3d> directionCoord(other.data());
        Eigen::Map<Eigen::Vector3d> resultData(result.data());

        resultData = newCoordFrame*directionCoord;

        return result;
    }

    ClassicalAcc Rotation::changeCoordFrameOf(const ClassicalAcc &other) const
    {
        ClassicalAcc result;
        result = RotationRaw::changeCoordFrameOf(other);

        return result;
    }

    RotationalInertiaRaw Rotation::changeCoordFrameOf(const RotationalInertiaRaw &other) const
    {
        RotationalInertiaRaw result;

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

    void Rotation::fromQuaternion(const iDynTree::Vector4& _quaternion)
    {
        Eigen::Map<const Eigen::Vector4d> quaternionIn(_quaternion.data());
        //normalize input quaternion
        Eigen::Vector4d quaternion(quaternionIn);
        quaternion.normalize();

        //to avoid memory allocation "unroll" the summation of Rodrigues' Formula
        // R = I3 + 2s S(r) + 2S(r)^2,

        //The square of S(r) is symmetric, thus the diagonal elements are
        //filled only by that part
        (*this)(0,0) = 1 -2 * (quaternion(3) * quaternion(3) + quaternion(2) * quaternion(2));
        (*this)(1,1) = 1 -2 * (quaternion(3) * quaternion(3) + quaternion(1) * quaternion(1));
        (*this)(2,2) = 1 -2 * (quaternion(2) * quaternion(2) + quaternion(1) * quaternion(1));

        //The off diagonal elements are filled by
        //(symmetrically) from the S(r)^2
        //(antisymmetrically) from the S(r)
        //Symmetric part
        (*this)(0,1) = (*this)(1,0) = 2 * quaternion(1) * quaternion(2);
        (*this)(0,2) = (*this)(2,0) = 2 * quaternion(1) * quaternion(3);
        (*this)(1,2) = (*this)(2,1) = 2 * quaternion(2) * quaternion(3);

        //antisymmetric part
        double r01, r02, r12;

        r01 = 2 * quaternion(0) * (-1) * quaternion(3);
        r02 = 2 * quaternion(0) * (+1) * quaternion(2);
        r12 = 2 * quaternion(0) * (-1) * quaternion(1);

        (*this)(0,1) += r01;
        (*this)(0,2) += r02;
        (*this)(1,2) += r12;

        (*this)(1,0) -= r01;
        (*this)(2,0) -= r02;
        (*this)(2,1) -= r12;
    }

    void Rotation::getRPY(double& r, double& p, double& y) const
    {
        Eigen::Map<const Matrix3dRowMajor> R(m_data);


        if (R(2,0)<1.0)
        {
            if (R(2,0)>-1.0)
            {
                r=atan2(R(2,1),R(2,2));
                p=asin(-R(2,0));
                y=atan2(R(1,0),R(0,0));
            }
            else
            {
                // Not a unique solution
                r=0.0;
                p=M_PI/2.0;
                y=-atan2(-R(1,2),R(1,1));
            }
        }
        else
        {
            // Not a unique solution
            r=0.0;
            p=-M_PI/2.0;
            y=atan2(-R(1,2),R(1,1));
        }
    }

    iDynTree::Vector3 Rotation::asRPY() const
    {
        // get r, p, y values
        double r,p,y;
        this->getRPY(r,p,y);

        // return the result
        double vectorRPY[3] = {r,p,y};
        return iDynTree::Vector3(vectorRPY,3);
    }

    bool Rotation::getQuaternion(iDynTree::Vector4& quaternion) const
    {
        return getQuaternion(quaternion(0), quaternion(1), quaternion(2), quaternion(3));
    }

    bool Rotation::getQuaternion(double &s, double &r1, double &r2, double &r3) const
    {
        Eigen::Map<const Matrix3dRowMajor> R(m_data);

        //Taken from "Contributions au contrôle automatique de véhicules aériens"
        //PhD thesis of "Minh Duc HUA"
        //INRIA Sophia Antipolis
        //www.isir.upmc.fr/files/2009THDR2323.pdf
        //Equation 3.9 (page 101)

        //Diagonal elements used only to find the maximum
        //the furthest value from zero
        //we use this value as denominator to find the other elements
        double q0 = ( R(0,0) + R(1,1) + R(2,2) + 1.0);
        double q1 = ( R(0,0) - R(1,1) - R(2,2) + 1.0);
        double q2 = (-R(0,0) + R(1,1) - R(2,2) + 1.0);
        double q3 = (-R(0,0) - R(1,1) + R(2,2) + 1.0);

        if (q0 < 0.0) q0 = 0.0;
        if (q1 < 0.0) q1 = 0.0;
        if (q2 < 0.0) q2 = 0.0;
        if (q3 < 0.0) q3 = 0.0;

        if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
            q0 = std::sqrt(q0);
            q1 = (R(2,1) - R(1,2)) / (2.0 * q0);
            q2 = (R(0,2) - R(2,0)) / (2.0 * q0);
            q3 = (R(1,0) - R(0,1)) / (2.0 * q0);
            q0 /= 2.0;
        } else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
            q1 = std::sqrt(q1);
            q0 = (R(2,1) - R(1,2)) / (2.0 * q1);
            q2 = (R(1,0) + R(0,1)) / (2.0 * q1);
            q3 = (R(2,0) + R(0,2)) / (2.0 * q1);
            q1 /= 2.0;
        } else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
            q2 = std::sqrt(q2);
            q0 = (R(0,2) - R(2,0)) / (2.0 * q2);
            q1 = (R(1,0) + R(0,1)) / (2.0 * q2);
            q3 = (R(1,2) + R(2,1)) / (2.0 * q2);
            q2 /= 2.0;
        } else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
            q3 = std::sqrt(q3);
            q0 = (R(1,0) - R(0,1)) / (2.0 * q3);
            q1 = (R(2,0) + R(0,2)) / (2.0 * q3);
            q2 = (R(1,2) + R(2,1)) / (2.0 * q3);
            q3 /= 2.0;
        } else {
            reportError("Rotation", "getQuaternion", "Quaternion numerically bad conditioned");
            return false;
        }

        //Here we impose that the leftmost nonzero element of the quaternion is positive
        double eps = 1e-7;
        double sign = 1.0;
        if (q0 > eps || q0 < -eps) {
            sign = q0 > 0 ? 1.0 : -1.0;
        } else if (q1 > eps || q1 < -eps) {
            sign = q1 > 0 ? 1.0 : -1.0;
        } else if (q2 > eps || q2 < -eps) {
            sign = q2 > 0 ? 1.0 : -1.0;
        } else if (q3 > eps || q3 < -eps) {
            sign = q3 > 0 ? 1.0 : -1.0;
        }

        q0 /= sign;
        q1 /= sign;
        q2 /= sign;
        q3 /= sign;

        double quaternionNorm = std::sqrt(q0 * q0 +
                                          q1 * q1 +
                                          q2 * q2 +
                                          q3 * q3);
        s  = q0 / quaternionNorm;
        r1 = q1 / quaternionNorm;
        r2 = q2 / quaternionNorm;
        r3 = q3 / quaternionNorm;
        return true;
    }

    iDynTree::Vector4 Rotation::asQuaternion() const
    {
        iDynTree::Vector4 quaternion;
        quaternion.zero();
        getQuaternion(quaternion);
        return quaternion;
    }


    AngularMotionVector3 Rotation::log() const
    {
        AngularMotionVector3 ret;

        Eigen::AngleAxisd aa(Eigen::Map<const Matrix3dRowMajor>(this->data()));

        // Implementation inspired from DART, see
        // https://github.com/dartsim/dart/pull/407/files
        // https://github.com/dartsim/dart/pull/334
        // https://github.com/dartsim/dart/issues/88
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

    Matrix3x3 Rotation::RotAxisDerivative(const Direction& direction, const double angle)
    {
        Matrix3x3 result;

        Eigen::Map<Matrix3dRowMajor> res(result.data());
        Eigen::Map<const Eigen::Vector3d> d(direction.data());
        Matrix3dRowMajor skewd = skew(d);

        res = skewd*cos(angle)+skewd*skewd*sin(angle);

        return result;
    }

    Rotation Rotation::RPY(const double roll, const double pitch, const double yaw)
    {
        return Rotation(RotationRaw::RPY(roll, pitch, yaw));
    }

    Matrix3x3 Rotation::RPYRightTrivializedDerivative(const double /*roll*/, const double pitch, const double yaw)
    {
        // See doc/symbolic/RPYExpressionReference.py

        Matrix3x3 map;

        double sp = std::sin(pitch);
        double cp = std::cos(pitch);
        double sy = std::sin(yaw);
        double cy = std::cos(yaw);

        map(0, 0) = cp*cy;
        map(1, 0) = sy*cp;
        map(2, 0) = -sp;
        map(0, 1) = -sy;
        map(1, 1) = cy;
        map(2, 1) = 0.0;
        map(0, 2) = 0.0;
        map(1, 2) = 0.0;
        map(2, 2) = 1.0;

        return map;
    }

    Matrix3x3 Rotation::RPYRightTrivializedDerivativeRateOfChange(const double /*roll*/, const double pitch, const double yaw, const double /*rollDot*/, const double pitchDot, const double yawDot)
    {
        Matrix3x3 map;

        double sp = std::sin(pitch);
        double cp = std::cos(pitch);
        double sy = std::sin(yaw);
        double cy = std::cos(yaw);

        map(0, 0) = -sp * cy * pitchDot - cp * sy * yawDot;
        map(1, 0) = -sp * sy * pitchDot + cy * cp * yawDot;
        map(2, 0) = -cp * pitchDot;
        map(0, 1) = -cy * yawDot;
        map(1, 1) = -sy * yawDot;
        map(2, 1) = 0.0;
        map(0, 2) = 0.0;
        map(1, 2) = 0.0;
        map(2, 2) = 0.0;

        return map;
    }

    Matrix3x3 Rotation::RPYRightTrivializedDerivativeInverse(const double /*roll*/, const double pitch, const double yaw)
    {
        // See doc/symbolic/RPYExpressionReference.py

        Matrix3x3 map;

        double cp = std::cos(pitch);
        double tp = std::tan(pitch);
        double sy = std::sin(yaw);
        double cy = std::cos(yaw);

        map(0, 0) = cy/cp;
        map(1, 0) = -sy;
        map(2, 0) = cy*tp;
        map(0, 1) = sy/cp;
        map(1, 1) = cy;
        map(2, 1) = sy*tp;
        map(0, 2) = 0.0;
        map(1, 2) = 0.0;
        map(2, 2) = 1.0;

        return map;
    }

    Matrix3x3 Rotation::RPYRightTrivializedDerivativeInverseRateOfChange(const double /*roll*/, const double pitch, const double yaw, const double /*rollDot*/, const double pitchDot, const double yawDot)
    {
        Matrix3x3 map;

        double sp = std::sin(pitch);
        double cp = std::cos(pitch);
        double sy = std::sin(yaw);
        double cy = std::cos(yaw);
        double tp = std::tan(pitch);

        map(0, 0) = (-sy * cp * yawDot + cy * sp * pitchDot) / std::pow(cp, 2);
        map(1, 0) = -cy * yawDot;
        map(2, 0) = -sy * tp * yawDot + cy * pitchDot / std::pow(cp, 2);
        map(0, 1) = (cy * cp * yawDot + sy * sp * pitchDot) / std::pow(cp, 2);
        map(1, 1) = -sy * yawDot;
        map(2, 1) = cy * tp * yawDot + sy * pitchDot / std::pow(cp, 2);
        map(0, 2) = 0.0;
        map(1, 2) = 0.0;
        map(2, 2) = 0.0;

        return map;
    }

    MatrixFixSize<4, 3> Rotation::QuaternionRightTrivializedDerivative(Vector4 quaternion)
    {
        MatrixFixSize<4, 3> outputMatrix;
        Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > map = iDynTree::toEigen(outputMatrix);
        map.topRows<1>() = -iDynTree::toEigen(quaternion).tail<3>().transpose();
        map.bottomRows<3>().setIdentity();
        map.bottomRows<3>() *= quaternion(0);
        map.bottomRows<3>() -= iDynTree::skew(iDynTree::toEigen(quaternion).tail<3>());
        map *= 0.5;
        return outputMatrix;
    }

    MatrixFixSize<3, 4> Rotation::QuaternionRightTrivializedDerivativeInverse(Vector4 quaternion)
    {
        MatrixFixSize<3, 4> outputMatrix;
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > map = iDynTree::toEigen(outputMatrix);

        map.setZero();
        map.leftCols<1>() = -iDynTree::toEigen(quaternion).tail<3>();
        map.rightCols<3>().setIdentity();
        map.rightCols<3>() *= iDynTree::toEigen(quaternion)(0);
        map.rightCols<3>() += iDynTree::skew(iDynTree::toEigen(quaternion).tail<3>());

        map *= 2;
        return outputMatrix;
    }

    Rotation Rotation::Identity()
    {
        return RotationRaw::Identity();
    }

    Rotation Rotation::RotationFromQuaternion(const iDynTree::Vector4& _quaternion)
    {
        //Taken from "Contributions au contrôle automatique de véhicules aériens"
        //PhD thesis of "Minh Duc HUA"
        //INRIA Sophia Antipolis
        //Equation 3.8 (page 101)

        // Rodriques' formula
        // R = I3 + 2s S(r) + 2S(r)^2,
        Rotation _rotation;
        _rotation.fromQuaternion(_quaternion);
        return _rotation;
    }

    Matrix3x3 Rotation::leftJacobian(const AngularMotionVector3& omega)
    {
        iDynTree::Matrix3x3 J;
        auto I3 = Eigen::MatrixXd::Identity(3, 3);
        using iDynTree::toEigen;
        double norm = toEigen(omega).norm();

        if (iDynTree::checkDoublesAreEqual(norm, 0.0))
        {
            toEigen(J) = I3;
            return J;
        }

        double c{std::cos(norm)};
        double s{std::sin(norm)};

        double alpha1{(s/norm)};
        double alpha2{(1 - c)/norm};
        double alpha3{(1 - (s/norm))};

        Vector3 phi;
        toEigen(phi) = toEigen(omega);
        toEigen(phi).normalize();

        Matrix3x3 phi_cross;
        toEigen(phi_cross) = skew(toEigen(phi));

        toEigen(J) = alpha1*I3 + alpha2*toEigen(phi_cross) + alpha3*toEigen(phi)*toEigen(phi).transpose();
        return J;
    }

    Matrix3x3 Rotation::leftJacobianInverse(const AngularMotionVector3& omega)
    {
        iDynTree::Matrix3x3 Jinv;
        auto I3 = Eigen::MatrixXd::Identity(3, 3);
        using iDynTree::toEigen;
        double norm = toEigen(omega).norm();

        if (iDynTree::checkDoublesAreEqual(norm, 0.0))
        {
            toEigen(Jinv) = I3;
            return Jinv;
        }

        double normovertwo{norm/2.0};
        double c{std::cos(normovertwo)};
        double s{std::sin(normovertwo)};
        double cot{c/s};

        double alpha1{(normovertwo*cot)};
        double alpha2{(-normovertwo)};
        double alpha3{(1 - alpha1)};

        Vector3 phi;
        toEigen(phi) = toEigen(omega);
        toEigen(phi).normalize();

        Matrix3x3 phi_cross;
        toEigen(phi_cross) = skew(toEigen(phi));

        toEigen(Jinv) = alpha1*I3  + alpha2*toEigen(phi_cross) + alpha3*toEigen(phi)*toEigen(phi).transpose();
        return Jinv;
    }

    std::string Rotation::toString() const
    {
        std::stringstream ss;

        ss << RotationRaw::toString();

        return ss.str();
    }

    std::string Rotation::reservedToString() const
    {
        return this->toString();
    }


}
