/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */

#ifndef _DIRL_SPARSE_NUMERICAL_BASE_PARAMETERS_
#define _DIRL_SPARSE_NUMERICAL_BASE_PARAMETERS_

#include <kdl/jntarray.hpp>

#include <kdl_codyco/undirectedtree.hpp>
#include "dynamicRegressorGenerator.hpp"

namespace KDL {
namespace CoDyCo {
namespace Regressors { 


     /**
     * Algorithm under development
     */
    
    //int computeSparseNumericalIdentifiableSubspace(const DynamicRegressorGenerator & regr_gen, Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 1000, const bool verbose = false);

    
    /*
    int computeForwardSparseNumericalIdentifiableSubspace(const DynamicRegressorGenerator & regr_gen, Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 1000, const bool verbose = false);
    */
}

}

}
#endif
