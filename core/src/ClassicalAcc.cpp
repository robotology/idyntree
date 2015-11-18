/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/RotationRaw.h>

#include <Eigen/Dense>

namespace iDynTree
{

// \todo TODO avoid typedef duplication
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

ClassicalAcc::ClassicalAcc()
{

}

ClassicalAcc::ClassicalAcc(const double* in_data,
             const unsigned int in_size):
             Vector6(in_data, in_size)
{

}

ClassicalAcc::ClassicalAcc(const ClassicalAcc& other):
              Vector6(other.data(),6)
{

}


ClassicalAcc::~ClassicalAcc()
{

}

const ClassicalAcc& ClassicalAcc::changeCoordFrame(const RotationRaw& newCoordFrame)
{
    Eigen::Map<Vector6d> thisData(this->data());
    Eigen::Map<const Matrix3dRowMajor> rotData(newCoordFrame.data());

    thisData.segment<3>(0) = rotData*thisData.segment<3>(0);
    thisData.segment<3>(3) = rotData*thisData.segment<3>(3);

    return *this;
}

ClassicalAcc ClassicalAcc::Zero()
{
    return ClassicalAcc();
}


}