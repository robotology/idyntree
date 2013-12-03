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

namespace dirl
{
    int calculateEssentialParametersSubspace(DynamicRegressorGenerator & regressor_generator, 
                                             const IBatchDynamicDataset & dataset,
                                             Eigen::MatrixXd & essential_parameters_subspace, 
                                             const double tol) 
    {        
        if( regressor_generator.getNrOfParameters() <= 0 ) { return -2; }
        
        //Y^T*Y nrOfParams x nfOfParams matrix encoding the essential parameters subspace 
        Eigen::MatrixXd YTY(regressor_generator.getNrOfParameters(),regressor_generator.getNrOfParameters());
        
        Eigen::MatrixXd regressor;
        Eigen::VectorXd known_terms;
        
        DynamicSample sample;
        
        
        //Initial value is zero
        YTY.setZero();
        
        for(int i=0; i < dataset.getNrOfSamples(); i++ ) {
            bool status = dataset.getSample(i,sample);
            if( !status ) { return -3; }
            
            if( regressor_generator.getNrOfDOFs() != sample.getNrOfDOFs() ) { return -1; }

            
            //Set robot state and sensors
            regressor_generator.setRobotStateAndSensors(sample);
            
            //Get regressor
            regressor_generator.computeRegressor(regressor,known_terms);
            
            YTY += regressor.transpose()*regressor;
        }
        
        int status = getRowSpaceBasis(YTY,essential_parameters_subspace,tol,true);
        
        if( status != 0 ) { return -4; }
        
        return 0;
    }
}
