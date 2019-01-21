/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/AttitudeEstimatorUtils.h>

iDynTree::Vector3 crossVector(const iDynTree::Vector3& a, const iDynTree::Vector3& b)
{
    iDynTree::Vector3 out;
    iDynTree::toEigen(out) = iDynTree::toEigen(a).cross(iDynTree::toEigen(b));
    return out;
}

iDynTree::Matrix3x3 mapR3Toso3(const iDynTree::Vector3& omega)
{
    iDynTree::Matrix3x3 out;
    iDynTree::toEigen(out) = iDynTree::skew(iDynTree::toEigen(omega));
    return out;
}

bool checkSkewSymmetricity(const iDynTree::Matrix3x3& S)
{
    bool flag = false;
    if ( (iDynTree::toEigen(S) + iDynTree::toEigen(S).transpose()).isZero() )
    {
        flag = true;
    }
    return flag;
}

iDynTree::Vector3 mapso3ToR3(const iDynTree::Matrix3x3& S)
{
    iDynTree::Vector3 out;
    iDynTree::toEigen(out) = iDynTree::unskew(iDynTree::toEigen(S));
    return out;
}

double innerProduct(const iDynTree::Vector3 a, const iDynTree::Vector3& b)
{
    return iDynTree::toEigen(a).dot(iDynTree::toEigen(b));
}

double realPartOfQuaternion(const iDynTree::Quaternion& q)
{
    return q(0);
}

iDynTree::Vector3 imaginaryPartOfQuaternion(const iDynTree::Quaternion &q)
{
    iDynTree::Vector3 v;
    v(0) = q(1);
    v(1) = q(2);
    v(2) = q(3);
    return v;
}

iDynTree::Quaternion composeQuaternion(const iDynTree::Quaternion &q1, const iDynTree::Quaternion &q2)
{
    // const clock_t begin_time = clock();
    iDynTree::Quaternion out;
    double s1 = realPartOfQuaternion(q1);
    iDynTree::Vector3 v1 = imaginaryPartOfQuaternion(q1);

    double s2 = realPartOfQuaternion(q2);
    iDynTree::Vector3 v2 = imaginaryPartOfQuaternion(q2);

    iDynTree::Vector3 imagOut;
    iDynTree::toEigen(imagOut) = iDynTree::toEigen(v2)*s1 + iDynTree::toEigen(v1)*s2 + iDynTree::toEigen(crossVector(v1, v2));
    out(0) = s1*s2 - innerProduct(v1, v2);
    out(1) = imagOut(0);
    out(2) = imagOut(1);
    out(3) = imagOut(2);
    //std::cout << "average time : " << float( clock() - begin_time ) / CLOCKS_PER_SEC << " s." << std::endl;
    return out;
}

iDynTree::Matrix4x4 mapofYQuaternionToXYQuaternion(const iDynTree::Quaternion &x)
{
    // unitary matrix structure represented by 2 complex numbers z1 = q0+iq1 and z2 = q2+iq3
    std::vector<double> v{x(0), -x(1), -x(2), -x(3),
                          x(1),  x(0), -x(3),  x(2),
                          x(2),  x(3),  x(0), -x(1),
                          x(3), -x(2),  x(1),  x(0)};
    return iDynTree::Matrix4x4(v.data(), 4, 4);
}

iDynTree::Quaternion composeQuaternion2(const iDynTree::Quaternion &q1, const iDynTree::Quaternion &q2)
{
    // this function is computationally faster than composeQuaternion
    // const clock_t begin_time = clock();
    iDynTree::Quaternion out;
    iDynTree::toEigen(out) = iDynTree::toEigen(mapofYQuaternionToXYQuaternion(q1))*iDynTree::toEigen(q2);
    //std::cout << "average time func2 : " << float( clock() - begin_time ) / CLOCKS_PER_SEC << " s." << std::endl;
    return out;
}

iDynTree::Quaternion pureQuaternion(const iDynTree::Vector3& bodyFixedFrameVelocityInInertialFrame)
{
    iDynTree::Quaternion p;
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

iDynTree::RPY quaternion2eulerRPY(const iDynTree::Quaternion &q)
{
    iDynTree::RPY rpy;
    //double q0squared{q(0)*q(0)};
    double q1squared{q(1)*q(1)};
    double q2squared{q(2)*q(2)};
    double q3squared{q(3)*q(3)};
    double q0q1{q(0)*q(1)};
    double q2q3{q(2)*q(3)};
    double q0q2{q(0)*q(2)};
    double q1q3{q(1)*q(3)};
    double q0q3{q(0)*q(3)};
    double q1q2{q(1)*q(2)};

    double roll, pitch, yaw;
    double sine_of_pitch = 2*(q0q2 - q1q3);
    if (fabs(sine_of_pitch) >= 1)
    {
        // if out of range use 90 degrees
        // source Wikipedia COnversion between quaternion and euler angles
        pitch = copysign(M_PI/2, sine_of_pitch);
    }
    else
    {
        pitch = asin(sine_of_pitch);
    }

    // avoiding singularities at pitch equals plus/minus 90 degrees
    if (checkDoublesApproximatelyEqual(pitch, M_PI/2, 0.01) || checkDoublesApproximatelyEqual(pitch, -M_PI/2, 0.01))
    {
        roll = atan2(q(1), q(0));
        yaw = 0.0;
    }
    else
    {
        roll = atan2(2*(q0q1+q2q3), 1 - (2*(q1squared + q2squared)));
        yaw = atan2(2*(q0q3+q1q2), 1 - (2*(q2squared + q3squared)));
    }

    rpy(0) = roll;
    rpy(1) = pitch;
    rpy(2) = yaw;

    return rpy;
}
