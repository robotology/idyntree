/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
