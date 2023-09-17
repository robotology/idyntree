// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/InertiaNonLinearParametrization.h>
#include <iDynTree/SpatialInertia.h>

#include <Eigen/Dense>

#include <iDynTree/EigenHelpers.h>

namespace iDynTree
{

/////////////////////////////////////////
// EIGEN HELPERS
//////////////////////////////////////////

/**
 * vec operator, as defined in the math literature.
 * The 3x3 input matrix is transformed in a 9-elements
 * vector using the column major serialization.
 */
Eigen::Matrix<double, 9, 1> vecColMajor(const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> & mat)
{
    Eigen::Matrix<double, 9, 1> vec;

    vec(0) = mat(0,0);
    vec(1) = mat(1,0);
    vec(2) = mat(2,0);
    vec(3) = mat(0,1);
    vec(4) = mat(1,1);
    vec(5) = mat(2,1);
    vec(6) = mat(0,2);
    vec(7) = mat(1,2);
    vec(8) = mat(2,2);

    return vec;
}

/**
 * vec operator, as defined in the math literature.
 * The 3x3 input matrix is transformed in a 9-elements
 * vector using the column major serialization.
 */
Eigen::Matrix<double, 3, 3, Eigen::RowMajor> unvecColMajor(const Eigen::Matrix<double, 9, 1> & vec)
{
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> mat;

    mat(0,0) = vec(0);
    mat(1,0) = vec(1);
    mat(2,0) = vec(2);
    mat(0,1) = vec(3);
    mat(1,1) = vec(4);
    mat(2,1) = vec(5);
    mat(0,2) = vec(6);
    mat(1,2) = vec(7);
    mat(2,2) = vec(8);

    return mat;
}


Eigen::Matrix<double, 6, 1> vech(const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> & mat)
{
    Eigen::Matrix<double, 6, 1> vec;

    vec(0) = mat(0,0);
    vec(1) = mat(0,1);
    vec(2) = mat(0,2);
    vec(3) = mat(1,1);
    vec(4) = mat(2,1);
    vec(5) = mat(2,2);

    return vec;
}

Eigen::Matrix<double, 3, 3> diag(const Eigen::Matrix<double, 3, 1> & vec)
{
    Eigen::Matrix<double, 3, 3> ret;

    ret.setZero();

    ret(0,0) = vec(0);
    ret(1,1) = vec(1);
    ret(2,2) = vec(2);

    return ret;
}

Eigen::Matrix<double, 3, 3> Delta(const unsigned int r, const unsigned int c)
{
    Eigen::Matrix<double, 3, 3> ret;

    ret.setZero();

    ret(r,c) = 1.0;

    return ret;
}

Eigen::Matrix<double, 3, 1> delta(const unsigned int r)
{
    Eigen::Matrix<double, 3, 1> ret;

    ret.setZero();

    ret(r) = 1.0;

    return ret;
}

void RigidBodyInertiaNonLinearParametrization::fromInertialParameters(const Vector10& inertialParams)
{
    SpatialInertia spatialInertia;
    spatialInertia.fromVector(inertialParams);
    fromRigidBodyInertia(spatialInertia);
}

void RigidBodyInertiaNonLinearParametrization::fromRigidBodyInertia(const SpatialInertia& inertia)
{
    using namespace Eigen;

    // We get the mass and the center of mass
    this->mass = inertia.getMass();

    this->com = inertia.getCenterOfMass();

    // We get the inertia at the COM
    RotationalInertia inertiaAtCOM = inertia.getRotationalInertiaWrtCenterOfMass();

    // We get the inertia at the principal axis using eigen
    JacobiSVD<Matrix<double,3,3,RowMajor> > eigenValuesSolver;

    // We check both the positive definitiveness of the matrix and the triangle
    // inequality by directly checking the central second moment of mass of the rigid body
    // In a nutshell, we have that:
    // Ixx = Cyy + Czz
    // Iyy = Cxx + Czz
    // Izz = Cxx + Cyy
    // so
    // Cxx = (Iyy + Izz - Ixx)/2
    // Cyy = (Ixx + Izz - Iyy)/2
    // Czz = (Ixx + Iyy - Izz)/2
    // Then all the condition boils down to:
    // Cxx >= 0 , Cyy >= 0 , Czz >= 0
    eigenValuesSolver.compute( toEigen(inertiaAtCOM) , Eigen::ComputeFullU | Eigen::ComputeFullV);

    double Ixx = eigenValuesSolver.singularValues()(0);
    double Iyy = eigenValuesSolver.singularValues()(1);
    double Izz = eigenValuesSolver.singularValues()(2);

    double Cxx = (Iyy + Izz - Ixx)/2;
    double Cyy = (Ixx + Izz - Iyy)/2;
    double Czz = (Ixx + Iyy - Izz)/2;

    this->centralSecondMomentOfMass(0) = Cxx;
    this->centralSecondMomentOfMass(1) = Cyy;
    this->centralSecondMomentOfMass(2) = Czz;

    toEigen(this->link_R_centroidal) = eigenValuesSolver.matrixU();
}

SpatialInertia RigidBodyInertiaNonLinearParametrization::toRigidBodyInertia() const
{
    // Compute the inertia wrt to the center of mass
    RotationalInertia rotInertia;

    Eigen::Vector3d principalMomentOfInertiaAtCOM;

    principalMomentOfInertiaAtCOM(0) = this->centralSecondMomentOfMass(1) + this->centralSecondMomentOfMass(2);
    principalMomentOfInertiaAtCOM(1) = this->centralSecondMomentOfMass(0) + this->centralSecondMomentOfMass(2);
    principalMomentOfInertiaAtCOM(2) = this->centralSecondMomentOfMass(0) + this->centralSecondMomentOfMass(1);

    toEigen(rotInertia) = toEigen(this->link_R_centroidal)*
                          diag(principalMomentOfInertiaAtCOM)*
                          toEigen(this->link_R_centroidal.inverse());

    SpatialInertia retInertia;

    retInertia.fromRotationalInertiaWrtCenterOfMass(this->mass,this->com,rotInertia);

    return retInertia;
}

Vector16 RigidBodyInertiaNonLinearParametrization::asVectorWithRotationAsVec() const
{
    using namespace Eigen;

    Vector16 vectorization;

    vectorization(0) = this->mass;

    vectorization(1) = this->com(0);
    vectorization(2) = this->com(1);
    vectorization(3) = this->com(2);

    Map< Matrix<double, 9, 1> >(vectorization.data()+4) = vecColMajor(toEigen(this->link_R_centroidal));

    vectorization(13) = this->centralSecondMomentOfMass(0);
    vectorization(14) = this->centralSecondMomentOfMass(1);
    vectorization(15) = this->centralSecondMomentOfMass(2);



    return vectorization;
}

void RigidBodyInertiaNonLinearParametrization::fromVectorWithRotationAsVec(const Vector16& vectorization)
{
    using namespace Eigen;

    this->mass = vectorization(0);

    this->com(0) = vectorization(1);
    this->com(1) = vectorization(2);
    this->com(2) = vectorization(3);

    toEigen(this->link_R_centroidal) =  unvecColMajor(Map< const Matrix<double, 9, 1> >(vectorization.data()+4));

    this->centralSecondMomentOfMass(0) = vectorization(13);
    this->centralSecondMomentOfMass(1) = vectorization(14);
    this->centralSecondMomentOfMass(2) = vectorization(15);
}


Transform RigidBodyInertiaNonLinearParametrization::getLinkCentroidalTransform() const
{
    return Transform(this->link_R_centroidal,this->com);
}

bool RigidBodyInertiaNonLinearParametrization::isPhysicallyConsistent() const
{
    return (this->mass > 0) &&
           (this->centralSecondMomentOfMass(0) >= 0.0) &&
           (this->centralSecondMomentOfMass(1) >= 0.0) &&
           (this->centralSecondMomentOfMass(2) >= 0.0);
}

Matrix10x16 RigidBodyInertiaNonLinearParametrization::getGradientWithRotationAsVec() const
{
    using namespace Eigen;
    Matrix10x16 ret;

    Map< Matrix<double, 10, 16, Eigen::RowMajor> > retEigen = toEigen(ret);

    retEigen.setZero();

    // Set the mass regressor
    retEigen(0,0) = 1.0;

    // Set the first moment of mass regressor
    Vector3d comEigen = toEigen(this->com);
    retEigen.block<3,1>(1,0) = comEigen;

    retEigen(1,1) = this->mass;
    retEigen(2,2) = this->mass;
    retEigen(3,3) = this->mass;

    double cx2 = comEigen(0)*comEigen(0);
    double cy2 = comEigen(1)*comEigen(1);
    double cz2 = comEigen(2)*comEigen(2);

    double cx = comEigen(0);
    double cy = comEigen(1);
    double cz = comEigen(2);
    double m = this->mass;

    // Set inertia elements regressor

    // Regressor wrt to mass
    retEigen(4,0) = cy2 + cz2;
    retEigen(5,0) = -cx*cy;
    retEigen(6,0) = -cx*cz;
    retEigen(7,0) = cx2 + cz2;
    retEigen(8,0) = -cy*cz;
    retEigen(9,0) = cx2 + cy2;

    // Regressor wrt to COM
    Matrix<double, 6, 3> regrWrtCOM;

    regrWrtCOM <<     0, 2*m*cy , 2*m*cz,
                  -m*cy,  -m*cx , 0,
                  -m*cz,      0 , -m*cx,
                 2*m*cx,      0 , 2*m*cz,
                      0,  -m*cz , -m*cy,
                 2*m*cx, 2*m*cy , 0;

    retEigen.block<6,3>(4,1) = regrWrtCOM;

    // Regressor wrt link_R_centroidal rotation
    Eigen::Vector3d principalMomentOfInertiaAtCOM;

    principalMomentOfInertiaAtCOM(0) = this->centralSecondMomentOfMass(1) + this->centralSecondMomentOfMass(2);
    principalMomentOfInertiaAtCOM(1) = this->centralSecondMomentOfMass(0) + this->centralSecondMomentOfMass(2);
    principalMomentOfInertiaAtCOM(2) = this->centralSecondMomentOfMass(0) + this->centralSecondMomentOfMass(1);

    Eigen::Matrix3d R = toEigen(this->link_R_centroidal);

    for(unsigned int c = 0; c < 3; c++ )
    {
        for(unsigned int r = 0; r < 3; r++ )
        {
            retEigen.block<6,1>(4,4+3*c+r) =
                vech(   Delta(r,c)*diag(principalMomentOfInertiaAtCOM)*R.transpose()
                      +          R*diag(principalMomentOfInertiaAtCOM)*Delta(r,c).transpose());

        }
    }

    Eigen::Matrix3d PPT;
    PPT.setZero();

    PPT(0,1) = PPT(0,2) = 1.0;
    PPT(1,0) = PPT(1,2) = 1.0;
    PPT(2,0) = PPT(2,1) = 1.0;


    for(unsigned int l = 0; l < 3; l++ )
    {
        retEigen.block<6,1>(4,13+l) = vech(R*diag(PPT*delta(l))*R.transpose());
    }

    return ret;
}




}
