// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/AttitudeEstimatorUtils.h>
#include <vector>
#include <cmath>

bool checkValidMeasurement(const iDynTree::Vector3& in, const std::string& measurement_type, bool check_also_zero_vector)
{
    if (check_also_zero_vector)
    {
        if (isZeroVector(in))
        {
            iDynTree::reportError("AttitudeEstimator", "checkValidMeasurement",
                                  (measurement_type + " measurements are invalid. Expecting a non-zero vector.").c_str());
            return false;
        }
    }

    if (isVectorNaN(in))
    {
        iDynTree::reportError("AttitudeEstimator", "checkValidMeasurement",
                              (measurement_type + " measurements are invalid. Has NaN elements.").c_str());
        return false;
    }

    return true;
}


bool getUnitVector(const iDynTree::Vector3& in, iDynTree::Vector3& out)
{
    using iDynTree::toEigen;

    double norm{toEigen(in).norm()};
    if (norm == 0)
    {
        return false;
    }

    out = in;
    toEigen(out).normalize();
    return true;
}

bool isVectorNaN(const iDynTree::Vector3& vec)
{
    for (size_t i = 0; i < vec.size(); i++)
    {
        if (std::isnan(vec(i)))
        {
            return true;
        }
    }
    return false;
}

bool isZeroVector(const iDynTree::Vector3& vec)
{
    for (size_t i = 0; i < vec.size(); i++)
    {
        if (vec(i) != 0)
        {
            return false;
        }
    }
    return true;
}

iDynTree::Vector3 crossVector(const iDynTree::Vector3& a, const iDynTree::Vector3& b)
{
    using iDynTree::toEigen;

    iDynTree::Vector3 out;
    toEigen(out) = toEigen(a).cross(toEigen(b)); // to be read as out = a.cross(b)

    return out;
}

iDynTree::Matrix3x3 mapR3Toso3(const iDynTree::Vector3& omega)
{
    using iDynTree::toEigen;

    iDynTree::Matrix3x3 out;
    toEigen(out) = iDynTree::skew(toEigen(omega)); // to be read as out = skew(omega)

    return out;
}

bool checkSkewSymmetricity(const iDynTree::Matrix3x3& S)
{
    using iDynTree::toEigen;

    bool flag = false;
    if ( (toEigen(S) + toEigen(S).transpose()).isZero() ) // to be read as (S + S.transpose()).isZero()
    {
        flag = true;
    }

    return flag;
}

iDynTree::Vector3 mapso3ToR3(const iDynTree::Matrix3x3& S)
{
    using iDynTree::toEigen;

    iDynTree::Vector3 out;
    toEigen(out) = iDynTree::unskew(toEigen(S));  // to be read as out = unskew(S)

    return out;
}

double innerProduct(const iDynTree::Vector3 a, const iDynTree::Vector3& b)
{
    using iDynTree::toEigen;

    return toEigen(a).dot(toEigen(b)); // to be read as a.dot(b)
}

double realPartOfQuaternion(const iDynTree::UnitQuaternion& q)
{
    return q(0);
}

iDynTree::Vector3 imaginaryPartOfQuaternion(const iDynTree::UnitQuaternion &q)
{
    iDynTree::Vector3 v;
    v(0) = q(1);
    v(1) = q(2);
    v(2) = q(3);
    return v;
}

iDynTree::UnitQuaternion composeQuaternion(const iDynTree::UnitQuaternion &q1, const iDynTree::UnitQuaternion &q2)
{
    using iDynTree::toEigen;

    double s1 = realPartOfQuaternion(q1);
    iDynTree::Vector3 v1 = imaginaryPartOfQuaternion(q1);

    double s2 = realPartOfQuaternion(q2);
    iDynTree::Vector3 v2 = imaginaryPartOfQuaternion(q2);

    iDynTree::Vector3 imagOut;
    toEigen(imagOut) = toEigen(v2)*s1 + toEigen(v1)*s2 + toEigen(crossVector(v1, v2)); // to be read as imagOut = v2*s1 + v1*s2 + crossVector(v1, v2)

    iDynTree::UnitQuaternion out;
    out(0) = s1*s2 - innerProduct(v1, v2);
    out(1) = imagOut(0);
    out(2) = imagOut(1);
    out(3) = imagOut(2);

    return out;
}

iDynTree::Matrix4x4 mapofYQuaternionToXYQuaternion(const iDynTree::UnitQuaternion &x)
{
    // unitary matrix structure represented by 2 complex numbers z1 = q0+iq1 and z2 = q2+iq3
    std::vector<double> v{x(0), -x(1), -x(2), -x(3),
                          x(1),  x(0), -x(3),  x(2),
                          x(2),  x(3),  x(0), -x(1),
                          x(3), -x(2),  x(1),  x(0)};
    return iDynTree::Matrix4x4(v.data(), 4, 4);
}

iDynTree::UnitQuaternion composeQuaternion2(const iDynTree::UnitQuaternion &q1, const iDynTree::UnitQuaternion &q2)
{
    // this function is computationally faster than composeQuaternion
    using iDynTree::toEigen;

    iDynTree::UnitQuaternion out;
    toEigen(out) = toEigen(mapofYQuaternionToXYQuaternion(q1))*toEigen(q2); // to be read as out = mapofYQuaternionToXYQuaternion(q1)*q2

    return out;
}

iDynTree::UnitQuaternion pureQuaternion(const iDynTree::Vector3& bodyFixedFrameVelocityInInertialFrame)
{
    iDynTree::UnitQuaternion p;
    p(0) = 0;
    p(1) = bodyFixedFrameVelocityInInertialFrame(0);
    p(2) = bodyFixedFrameVelocityInInertialFrame(1);
    p(3) = bodyFixedFrameVelocityInInertialFrame(2);
    return p;
}

// method definition copied from The art of programming by Knuth
bool checkDoublesApproximatelyEqual(double val1, double val2, double tol)
{
    return fabs(val1 - val2) <= ( (fabs(val1) < fabs(val2) ? fabs(val2) : fabs(val1)) * tol );
}

