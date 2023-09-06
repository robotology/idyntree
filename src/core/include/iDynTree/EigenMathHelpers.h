// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_EIGEN_MATH_HELPERS_H
#define IDYNTREE_EIGEN_MATH_HELPERS_H

#include <Eigen/Dense>

namespace iDynTree
{

// Methods imported from codyco commons, see the documentation
// for them in
// https://github.com/robotology-playground/codyco-commons/blob/master/include/codyco/MathUtils.h#L48
template <typename MapType>
void pseudoInverse_helper2(const MapType& A,
                         Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                         MapType& Apinv,
                         double tolerance,
                         unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV)
{
    using namespace Eigen;

    if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
    svdDecomposition.compute(A, computationOptions);

    typename JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues = svdDecomposition.singularValues();
    int singularValuesSize = singularValues.size();
    int rank = 0;
    for (int idx = 0; idx < singularValuesSize; idx++) {
        if (tolerance > 0 && singularValues(idx) > tolerance) {
            singularValues(idx) = 1.0 / singularValues(idx);
            rank++;
        } else {
            singularValues(idx) = 0.0;
        }
    }

    //equivalent to this U/V matrix in case they are computed full
    Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();
}


template <typename MapType>
void pseudoInverse_helper1(const MapType& A,
                          Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                                 MapType& Apinv,
                   double tolerance,
                   unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV)
{
    using namespace Eigen;
    pseudoInverse_helper2(A, svdDecomposition, Apinv, tolerance, computationOptions);
}

template <typename MapType>
void pseudoInverse(const MapType A,
                         MapType Apinv,
                   double tolerance,
                   unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV)

{
    Eigen::JacobiSVD<Eigen::MatrixXd> svdDecomposition(A.rows(), A.cols());
    pseudoInverse_helper1(A, svdDecomposition, Apinv, tolerance, computationOptions);
}

}

#endif
