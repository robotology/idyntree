/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 * 
 * The development of this software was supported by the FP7 EU project 
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b)) 
 * http://www.codyco.eu
 */
 
#include <dirl/essentialParameters.hpp>
#include <dirl/dirl_utils.hpp>
#include <iostream>

namespace dirl
{
    
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
                                             Eigen::VectorXd & sigma,
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
