/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef DIRL_ESSENTIAL_PARAMETERS_HPP
#define DIRL_ESSENTIAL_PARAMETERS_HPP

#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>
#include <kdl_codyco/regressors/DynamicDatasetInterfaces.hpp>

namespace KDL {
namespace CoDyCo {
namespace Regressors {
    /**
     * For a given regressor and a given dataset, calculate the essential parameters
     *
     * The essential parameters are defined in:
     * PHAM, C. M.; GAUTIER, M. Essential parameters of robots.
     * In: Decision and Control, 1991., Proceedings of the 30th IEEE Conference on. IEEE, 1991. p. 2769-2774.
     *
     * @return 0 if all went well, -1 if the size of the regressor and the dataset does not match
     */
    int calculateEssentialParametersSubspace(DynamicRegressorGenerator & regressor,
                                             const IBatchDynamicDataset & dataset,
                                             Eigen::MatrixXd & essential_parameter_basis,
                                             const double tol = 1e-6);
    
    int calculateEssentialParametersSubspace(DynamicRegressorGenerator & regressor,
                                             const IBatchDynamicDataset & dataset,
                                             Eigen::MatrixXd & essential_parameter_basis,
                                             Eigen::VectorXd & sigma,
                                             const double tol = 1e-6);
}

}

}

#endif
