/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/MatrixFixSize.h>

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <kdl_codyco/KDLConversions.h>
#include <kdl_codyco/regressor_utils.hpp>

#include <iCub/iDynTree/yarp_kdl.h>

#include <iDynTree/Core/Transform.h>

#include <yarp/sig/Matrix.h>

using namespace iDynTree;

int main()
{
    // Create a random iDynTree frame
    Transform trans = getRandomTransform();

    Matrix6x6 adj_idyntree = trans.asAdjointTransform();
    yarp::sig::Matrix adj_yarp_kdl_inyarp = KDLtoYarp_twist(ToKDL(trans));
    Matrix6x6 adj_yarp_kdl;
    toEigen(adj_yarp_kdl) = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor> >(adj_yarp_kdl_inyarp.data());
    Matrix6x6 adj_eigen_kdl;
    toEigen(adj_eigen_kdl) = KDL::CoDyCo::TwistTransformationMatrix(ToKDL(trans));

    std::cerr << "Adjoint matrix: "<< std::endl;
    std::cerr << "iDynTree : \n" << adj_idyntree.toString() << std::endl;
    std::cerr << "YARP     : \n" << adj_yarp_kdl.toString() << std::endl;
    std::cerr << "Eigen    : \n" << adj_eigen_kdl.toString() << std::endl;

    Matrix6x6 adj_wrench_idyntree = trans.asAdjointTransformWrench();
    yarp::sig::Matrix adj_wrench_yarp_kdl_inyarp = KDLtoYarp_wrench(ToKDL(trans));
    Matrix6x6 adj_wrench_yarp_kdl;
    toEigen(adj_wrench_yarp_kdl) = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor> >(adj_wrench_yarp_kdl_inyarp.data());

    Matrix6x6 adj_wrench_eigen_kdl;
    toEigen(adj_wrench_eigen_kdl) = KDL::CoDyCo::WrenchTransformationMatrix(ToKDL(trans));

    std::cerr << "Wrench Adjoint matrix: "<< std::endl;
    std::cerr << "iDynTree : \n" << adj_wrench_idyntree.toString() << std::endl;
    std::cerr << "YARP     : \n" << adj_wrench_yarp_kdl.toString() << std::endl;
    std::cerr << "Eigen    : \n" << adj_wrench_eigen_kdl.toString() << std::endl;

    ASSERT_EQUAL_MATRIX(adj_idyntree,adj_yarp_kdl);
    ASSERT_EQUAL_MATRIX(adj_idyntree,adj_eigen_kdl);

    ASSERT_EQUAL_MATRIX(adj_wrench_idyntree,adj_wrench_yarp_kdl);
    ASSERT_EQUAL_MATRIX(adj_wrench_idyntree,adj_wrench_eigen_kdl);

    return EXIT_SUCCESS;
}



