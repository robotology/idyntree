/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
 
#include "essentialParameters.hpp"
#include "dirl_utils.hpp"
#include <iostream>

namespace KDL {
namespace CoDyCo {
namespace Regressors {   
    
    int calculateEssentialParametersSubspace(DynamicRegressorGenerator & regressor_generator, 
                                             const IBatchDynamicDataset & dataset,
                                             Eigen::MatrixXd & essential_parameters_subspace, 
                                             const double tol
                                            )
    {
        Eigen::VectorXd dummy;
        return calculateEssentialParametersSubspace(regressor_generator,dataset,essential_parameters_subspace,dummy,tol);
    }
    
    int calculateEssentialParametersSubspace(DynamicRegressorGenerator & regressor_generator, 
                                             const IBatchDynamicDataset & dataset,
                                             Eigen::MatrixXd & essential_parameters_subspace, 
                                             Eigen::VectorXd & /*sigma*/,
                                             const double tol
                                            ) 
    {        
        if( regressor_generator.getNrOfParameters() <= 0 ) { std::cerr << "calculateEssentialParametersSubspace error: regressor_generator not consistent" << std::endl; return -2; }
        
        //Y^T*Y nrOfParams x nfOfParams matrix encoding the essential parameters subspace 
        Eigen::MatrixXd YTY(regressor_generator.getNrOfParameters(),regressor_generator.getNrOfParameters());
        
        Eigen::MatrixXd regressor(regressor_generator.getNrOfOutputs(),regressor_generator.getNrOfParameters());
        Eigen::VectorXd known_terms(regressor_generator.getNrOfOutputs());
        
        DynamicSample sample;
        
        
        //Initial value is zero
        YTY.setZero();
        
        for(int i=0; i < dataset.getNrOfSamples(); i++ ) {
            bool status = dataset.getSample(i,sample);
            if( !status ) { return -3; }
            
            if( regressor_generator.getNrOfDOFs() != sample.getNrOfDOFs() ) {
                std::cerr << "calculateEssentialParametersSubspace error: mismatch between regressor generator ( " 
                          << regressor_generator.getNrOfDOFs() << " dofs ) and data sample (" << sample.getNrOfDOFs() << " dofs ) " <<std::endl;
                return -1;
                
            }

            
            //Set robot state and sensors
            regressor_generator.setRobotStateAndSensors(sample);
            
            //Get regressor
            regressor_generator.computeRegressor(regressor,known_terms);
            
            YTY += regressor.transpose()*regressor;
        }
        
        int status = getRowSpaceBasis(YTY,essential_parameters_subspace,tol,true);
        
        if( status != 0 ) { std::cerr << "calculateEssentialParametersSubspace error: getRowSpaceBasis failed" << std::endl;  return -4; }
        
        return 0;
    }
}

}

}
