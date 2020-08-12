/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <iostream>
#include <cmath>

iDynTree::CubicSpline::CubicSpline()
:m_v0(0)
,m_vf(0)
,m_a0(0)
,m_af(0)
,m_areCoefficientsUpdated{false}
{
    m_coefficients.clear();
    m_velocities.resize(0);
    m_time.resize(0);
    m_y.resize(0);
    m_T.resize(0);
}

iDynTree::CubicSpline::CubicSpline(unsigned int buffersDimension)
    :m_coefficients(buffersDimension - 1)
    ,m_velocities(buffersDimension)
    ,m_time(buffersDimension)
    ,m_y(buffersDimension)
    ,m_T(buffersDimension - 1)
    ,m_v0(0)
    ,m_vf(0)
    ,m_a0(0)
    ,m_af(0)
    ,m_areCoefficientsUpdated{false}
{
}

bool iDynTree::CubicSpline::setData(const iDynTree::VectorDynSize& time, const iDynTree::VectorDynSize& yData)
{
    if((time.size() == 0) && (yData.size() == 0)){
        std::cerr << "[ERROR][CUBICSPLINE] The input data are empty!" << std::endl;
        return false;
    }

    if(time.size() != yData.size()){
        std::cerr << "[ERROR][CUBICSPLINE] The input data are expected to have the same dimension: xData = " << time.size() << ", yData = " << yData.size() << "." <<std::endl;
        return false;
    }

    if(time.size() < 2){
        std::cerr << "[ERROR][CUBICSPLINE] At least two data points are needed to compute the spline." << std::endl;
        return false;
    }

    m_time.resize(time.size());
    m_y.resize(yData.size());

    m_coefficients.resize(time.size()-1);
    m_velocities.resize(time.size());

    m_T.resize(time.size()-1);

    m_time = time;

    m_y = yData;

    m_areCoefficientsUpdated = false;

    return true;
}

bool iDynTree::CubicSpline::computeCoefficients()
{
    // the coefficients are updated. No need to recompute them
    if(m_areCoefficientsUpdated){
        return true;
    }

    m_velocities(0) = m_v0;
    m_velocities(m_velocities.size() - 1) = m_vf;

    if(!this->computePhasesDuration())
        return false;

    if(m_velocities.size() > 2){
        if(!this->computeIntermediateVelocities())
            return false;
    }

    for(size_t i = 0; i < m_coefficients.size(); ++i){
        m_coefficients[i](0) = m_y(i);
        m_coefficients[i](1) = m_velocities(i);
        m_coefficients[i](2) = ( 3*(m_y(i+1) - m_y(i))/m_T(i) - 2*m_velocities(i) - m_velocities(i+1) )/m_T(i);
        m_coefficients[i](3) = ( 2*(m_y(i) - m_y(i+1))/m_T(i) + m_velocities(i) + m_velocities(i+1) )/std::pow(m_T(i), 2);
    }

    // The coefficients are now updated.
    m_areCoefficientsUpdated = true;

    return true;
}

bool iDynTree::CubicSpline::computeIntermediateVelocities()
{
    Eigen::SparseMatrix<double> A(m_velocities.size()-2, m_velocities.size()-2);
    std::vector<Eigen::Triplet<double> > A_triplets;

    //Eigen::MatrixXd A(m_velocities.size()-2, m_velocities.size()-2);
    Eigen::VectorXd b(m_velocities.size()-2);

    //A.setZero();

    //A(0,0) = 2*(m_T(0) + m_T(1));
    A_triplets.push_back(Eigen::Triplet<double>(0, 0, 2*(m_T(0) + m_T(1))));
    b(0) = ( std::pow(m_T(0), 2)*(m_y(2) - m_y(1)) + std::pow(m_T(1), 2)*(m_y(1) - m_y(0)) )*3/(m_T(0)*m_T(1)) - m_T(1)*m_velocities(0);

    if(m_velocities.size() > 3){
        //A(0,1) = m_T(0);
        A_triplets.push_back(Eigen::Triplet<double>(0, 1, m_T(0)));
        for(int i = 1; i < A.rows()-1; ++i){
            //A(i,i-1) = m_T(i+1);
            A_triplets.push_back(Eigen::Triplet<double>(i, i-1, m_T(i+1)));
            //A(i,i) = 2*(m_T(i) + m_T(i+1));
            A_triplets.push_back(Eigen::Triplet<double>(i, i, 2*(m_T(i) + m_T(i+1))));
            //A(i,i+1) = m_T(i);
            A_triplets.push_back(Eigen::Triplet<double>(i, i+1, m_T(i)));

            b(i) = ( std::pow(m_T(i), 2)*(m_y(i+2) - m_y(i+1)) + std::pow(m_T(i+1), 2)*(m_y(i+1) - m_y(i)) )*3/(m_T(i)*m_T(i+1));
        }
        size_t T = m_T.size() - 1;
        //A.bottomRightCorner<1,2>() << m_T(T), 2*(m_T(T) + m_T(T-1));
        A_triplets.push_back(Eigen::Triplet<double>(A.rows() - 1, A.cols() - 2,  m_T(T)));
        A_triplets.push_back(Eigen::Triplet<double>(A.rows() - 1, A.cols() - 1,  2*(m_T(T) + m_T(T-1))));
        b.tail<1>() << ( std::pow(m_T(T-1), 2)*(m_y(T+1) - m_y(T)) + std::pow(m_T(T), 2)*(m_y(T) - m_y(T-1)) )*3/(m_T(T-1)*m_T(T)) - m_T(T-1)*m_velocities(T+1);
    }
    //iDynTree::toEigen(m_velocities).segment(1, m_velocities.size()-2) = A.colPivHouseholderQr().solve(b);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    A.makeCompressed();
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<Eigen::SparseMatrix<double>::StorageIndex>> qrDecomposition;
    qrDecomposition.compute(A);
    iDynTree::toEigen(m_velocities).segment(1, m_velocities.size()-2) = qrDecomposition.solve(b);

    return true;
}

bool iDynTree::CubicSpline::computePhasesDuration()
{
    for (size_t i = 0; i < m_time.size() - 1; ++i){

        m_T(i) = m_time(i+1) - m_time(i);

        if(m_T(i) == 0){
            std::cerr << "[ERROR][CUBICSPLINE] Two consecutive points have the same time coordinate." << std::endl; //For stability purposes, the matrix below may not be invertible
            return false;
        }

        if(m_T(i) < 0){
            std::cerr << "[ERROR][CUBICSPLINE] The input points are expected to be consecutive, strictly increasing in the time variable." << std::endl; //For stability purposes
            return false;
        }

    }
    return true;
}

void iDynTree::CubicSpline::setInitialConditions(double initialVelocity, double initialAcceleration)
{
    m_v0 = initialVelocity;
    m_a0 = initialAcceleration;

    // The initial condition has been updated. The coefficients have to be recomputed.
    m_areCoefficientsUpdated = false;
}

void iDynTree::CubicSpline::setFinalConditions(double finalVelocity, double finalAcceleration)
{
    m_vf = finalVelocity;
    m_af = finalAcceleration;

    // The initial condition has been updated. The coefficients have to be recomputed.
    m_areCoefficientsUpdated = false;
}

double iDynTree::CubicSpline::evaluatePoint(double t)
{
    double velocity, acceleration;

    return evaluatePoint(t, velocity, acceleration);
}

double iDynTree::CubicSpline::evaluatePoint(double t, double &velocity, double &acceleration)
{
    if(m_time.size() == 0){
        std::cerr << "[ERROR][CUBICSPLINE] First you have to load data! The returned data should not be considered." << std::endl;
        return std::nan("");
    }

    // The coefficients are not updated. It's time to compute them.
    if(!m_areCoefficientsUpdated){
        if(!this->computeCoefficients()){
                std::cerr << "[ERROR][CUBICSPLINE] Unable to compute the internal coefficients of the cubic spline." << std::endl;
        return std::nan("");
        }
    }

    if( t < m_time(0) ){
        velocity = m_v0;
        acceleration = m_a0;
        return m_y(0);
    }

    if( t >= m_time(m_time.size()-1)){
        velocity = m_vf;
        acceleration = m_af;
        return m_y(m_y.size()-1);
    }

    size_t coeffIndex = 0;
    while( (coeffIndex < m_time.size()) && (t >= m_time(coeffIndex)) ){
        coeffIndex++;
    }
    coeffIndex--; //Actually we are interested in the last index for which t >= m_time(coeffIndex) holds

    iDynTree::Vector4 coeff = m_coefficients[coeffIndex];
    double dt = t - m_time(coeffIndex);

    double position = coeff(0) + coeff(1)*(dt) + coeff(2)*std::pow(dt,2) + coeff(3)*std::pow(dt,3);
    velocity = coeff(1) + 2*coeff(2)*(dt) + 3*coeff(3)*std::pow(dt,2);
    acceleration = 2*coeff(2) + 6*coeff(3)*(dt);
    return position;
}
