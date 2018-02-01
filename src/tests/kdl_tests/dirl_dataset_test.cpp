/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl_codyco/utils.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_codyco/regressors/DynamicDatasetFile.hpp>
#include <cstdlib>

using namespace KDL;
using namespace KDL::CoDyCo;
using namespace KDL::CoDyCo::Regressors;

int main(int argc, char ** argv)
{
    
    if( argc != 3 || argc != 5 ) {
        std::cout << "dirl_dataset_test: test of loading a dataset and checking it consistency" << std::endl;
        std::cout << "usage: ./dirl_dataset_test dataset.csv n_expected_samples or" << std::endl;
        std::cout << "usage: ./dirl_dataset_test dataset.csv n_expected_samples last_timestamp last_sample_position_of_last_dof" << std::endl;
    }
    
    int n_expected_samples = atof(argv[2]);
    
    std::string dataset_file_name(argv[1]);
    
    DynamicDatasetFile test_dataset;
    
    test_dataset.loadFromFile(dataset_file_name);
    
    if( test_dataset.getNrOfSamples() != n_expected_samples ) { 
        std::cerr << "dirl_dataset_test: the number of samples ( " << test_dataset.getNrOfSamples()
                  << " ) is not the one expected ( " << n_expected_samples << " ) " << std::endl;
        return EXIT_FAILURE;
    }
    
    if( argc == 5 ) {
        double tol = 1e-10;
        double expected_last_timestamp = atof(argv[3]);
        double expected_last_position_last_dof = atof(argv[4]);
        
        DynamicSample test_sample;
        
        test_dataset.getSample(test_dataset.getNrOfSamples()-1,test_sample);
        
        double last_timestamp = test_sample.getTimestamp();
        double last_position_last_dof = test_sample.getJointPosition().operator()(test_sample.getNrOfDOFs()-1);
        
        if( fabs(last_timestamp-expected_last_timestamp) > tol ) 
        {
            std::cerr << "dirl_dataset_test: the last_timestamp ( " << last_timestamp
                      << " ) is not the one expected ( " << expected_last_timestamp << " ) " << std::endl;
            return EXIT_FAILURE;
        }
        
        if( fabs(last_position_last_dof-expected_last_position_last_dof) > tol) 
        {
            std::cerr << "dirl_dataset_test: the last_position of the last dof ( " << last_position_last_dof
                      << " ) is not the one expected ( " << expected_last_position_last_dof << " ) " << std::endl;
            return EXIT_FAILURE;
        } 
        
    }
    
    std::cout << "dirl_dataset_test: exit success" << std::endl;
    
    return 0;
}
