/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenMathHelpers.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>

#include <yarp/sig/Matrix.h>
#include <yarp/math/SVD.h>

using namespace iDynTree;

#include <ctime>

/**
 * Return the current time in seconds, with respect
 * to an arbitrary point in time.
 */
double clockInSec()
{
    clock_t ret = clock();
    return ((double)ret)/((double)CLOCKS_PER_SEC);
}

void pinvInYARP(yarp::sig::Matrix & mat,
                const double tol,
                yarp::sig::Matrix & pinvMat,
                double & duration)
{
    double tic = clockInSec();
    yarp::math::pinv(mat,pinvMat,tol);
    double toc = clockInSec();
    duration = toc-tic;
}

void pinvInEigen(MatrixDynSize & mat,
                 const double tol,
                 MatrixDynSize & pinvMat,
                 double & duration)
{
    double tic = clockInSec();
    pseudoInverse(toEigen(mat),toEigen(pinvMat),tol);
    double toc = clockInSec();
    duration = toc-tic;
}

void pseudoInverseBenchmark(int rows, int cols, unsigned int nrOfTrials)
{
    std::cerr << "Benchmarking dynamics algorithms for a matrix of size " << rows << " times " << cols << std::endl;

    // Check
    double tolSVD = 1e-7;
    double tolCheck = 1e-5;

    // initialization variables
    MatrixDynSize mat(rows,cols);
    MatrixDynSize pinvMat(cols,rows);
    MatrixDynSize pinvMatComputedWithYarp(cols,rows);

    yarp::sig::Matrix matYarp(rows,cols);
    yarp::sig::Matrix pinvMatYarp(cols,rows);

    getRandomMatrix(mat);
    memcpy(matYarp.data(),mat.data(),mat.rows()*mat.cols()*sizeof(double));


    double tic,toc;
    double totalYarpDuration = 0.0;
    double totalEigenDuration = 0.0;
    for(unsigned int trial=0; trial < nrOfTrials; trial++ )
    {
        bool coin = getRandomBool();

        double yarpDuration, eigenDuration;
        if( coin )
        {
            pinvInEigen(mat,tolSVD,pinvMat,eigenDuration);
            pinvInYARP(matYarp,tolSVD,pinvMatYarp,yarpDuration);
        }
        else
        {
            pinvInYARP(matYarp,tolSVD,pinvMatYarp,yarpDuration);
            pinvInEigen(mat,tolSVD,pinvMat,eigenDuration);
        }

        totalYarpDuration += yarpDuration;
        totalEigenDuration += eigenDuration;

        memcpy(pinvMatComputedWithYarp.data(),pinvMatYarp.data(),pinvMatYarp.rows()*pinvMatYarp.cols()*sizeof(double));

        ASSERT_EQUAL_MATRIX(pinvMatComputedWithYarp,pinvMat);
    }

    // if the matrix is 6 \times 6, we compare the svd with the inversion of the transformation matrix
    if( rows == 6 && cols == 6 )
    {
        double totalAdjointInversionDuration = 0.0;
        for(unsigned int trial=0; trial < nrOfTrials; trial++ )
        {
            double tic, toc;

            Transform trans = getRandomTransform();

            tic = clockInSec();
            Matrix6x6 adjointMatrix = trans.inverse().asAdjointTransform();
            toc = clockInSec();

            totalAdjointInversionDuration += toc-tic;
        }
        std::cerr << "\tiDynTree adjoint transform average time " << (totalAdjointInversionDuration/nrOfTrials)*1e6 << " microseconds " << std::endl;
    }

    std::cerr << "\tYARP average time " << (totalYarpDuration/nrOfTrials)*1e6 << " microseconds" << std::endl;
    std::cerr << "\tEIGEN average time " << (totalEigenDuration/nrOfTrials)*1e6 << " microseconds" << std::endl;

    return;
}


int main()
{
    std::cout << "PseudoInverse benchmark, iDynTree built in " << IDYNTREE_CMAKE_BUILD_TYPE << " mode " << std::endl;
    int nrOfTrials = 100;
    size_t rows = 6;
    for(unsigned int cols = 3; cols < 30; cols++ )
    {
        pseudoInverseBenchmark(rows,cols,nrOfTrials);
    }

    return EXIT_SUCCESS;
}
