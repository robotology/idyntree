/*
 * Copyright (C) 2013 Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <fstream>
#include <iostream>
#include <sstream>

#include <dirl/dataset/DynamicDatasetFile.hpp>

namespace dirl {
   
DynamicDatasetFile::DynamicDatasetFile()
{
    
}

DynamicDatasetFile::~DynamicDatasetFile()
{
    
}

int DynamicDatasetFile::getNrOfSamples() const
{
    return dynamic_samples.size();
}

bool DynamicDatasetFile::getSample(int sample_n, DynamicSample & sample) const
{
    if( sample_n < 0 || sample_n >= getNrOfSamples() ) { return false; }
    sample = dynamic_samples[sample_n];
    return true;
}

double get_next_double(std::stringstream & ss,std::string & data_buffer) 
{
    getline(ss,data_buffer,',');
    return atof(data_buffer.c_str());
}

bool DynamicDatasetFile::loadFromFile(const std::string file_name, const bool append)
{
    std::string data_buffer;
    std::string line_buffer;
    
    std::ifstream csv_file;
    
    #ifndef NDEBUG
    bool verbose = true;
    #endif 
    
    csv_file.open (file_name.c_str(), std::ifstream::in);
    
    if( !csv_file ) {
        std::cerr << "DynamicDatasetFile error: it was not possible to open file " << file_name << std::endl;
        return false;
    }
    
    //First row: field names of the header (not useful)
    getline(csv_file,data_buffer);
    
    #ifndef NDEBUG
    std::cout << "DynamicDatasetFile::loadFromFile: Reading first row: " << data_buffer << std::endl; 
    #endif
    
    //Second row: //N_DOFS,N_MEASURED_TORQUES,N_MEASURED_WRENCHES,N_MEASURED_3AXIS_FT,N_ADDITIONAL_ME
    getline(csv_file,data_buffer,',');
    if( !append ) {
        nrOfDOFs = atoi(data_buffer.c_str());
    } else {
        if( nrOfDOFs != atoi(data_buffer.c_str()) ) {
            std::cerr << "DynamicDatasetFile error: it was not possible to append file " << file_name 
                      << " because it has a different number of DOFs " << std::endl;
            return false;
        }
    }
    
    #ifndef NDEBUG
    std::cout << "DynamicDatasetFile::loadFromFile: Reading nrOfDOFs: " << nrOfDOFs << std::endl; 
    #endif
    
    getline(csv_file,data_buffer,',');
    if( !append) {
        nrOfMeasuredTorques = atoi(data_buffer.c_str());
    } else {
        if( nrOfMeasuredTorques != atoi(data_buffer.c_str()) ) {
            std::cerr << "DynamicDatasetFile error: it was not possible to append file " << file_name  
                      << " because it has a different number of measured torques " << std::endl;
            return false;
        }
    }
    
    
    #ifndef NDEBUG
    std::cout << "DynamicDatasetFile::loadFromFile: Reading nrOfMeasuredTorques: " << nrOfMeasuredTorques << std::endl; 
    #endif
    
    getline(csv_file,data_buffer,',');
    if( !append) {
        nrOfMeasuredWrenches = atoi(data_buffer.c_str());
    } else {
        if( nrOfMeasuredWrenches != atoi(data_buffer.c_str()) ) {
            std::cerr << "DynamicDatasetFile error: it was not possible to append file " << file_name
                      << " because it has a different number of wrenches " << std::endl;
            return false;
        }
    }
        
    #ifndef NDEBUG
    std::cout << "DynamicDatasetFile::loadFromFile: Reading nrOfMeasuredWrenches: " << nrOfMeasuredWrenches << std::endl; 
    #endif
    
    getline(csv_file,data_buffer,',');
    if( !append) {
        nrOfMeasured3AxisFT = atoi(data_buffer.c_str());
    } else {
        if( nrOfMeasured3AxisFT != atoi(data_buffer.c_str()) ) {
            std::cerr << "DynamicDatasetFile error: it was not possible to append file " << file_name
                      << " because it has a different number of 3 axis ft measures " << std::endl;
            return false;
        }
    }    
    #ifndef NDEBUG
    std::cout << "DynamicDatasetFile::loadFromFile: Reading nrOfMeasured3AxisFT: " << nrOfMeasured3AxisFT << std::endl; 
    #endif
    
    getline(csv_file,data_buffer);
    
    #ifndef NDEBUG
    std::cout << "DynamicDatasetFile::loadFromFile: Reading rest of second line: " << data_buffer << std::endl; 
    #endif
    
    //Third row: field names of the data (not parsed)
    getline(csv_file,data_buffer);
    
    #ifndef NDEBUG
    std::cout << "DynamicDatasetFile::loadFromFile: Reading third line: " << data_buffer << std::endl; 
    #endif
    
    //All remaining rows are data
    if( !append ) {
        DynamicSample sample(nrOfDOFs,nrOfMeasuredTorques,nrOfMeasuredWrenches,nrOfMeasured3AxisFT);
        dynamic_samples.resize(0,sample);
    }
    
    #ifndef NDEBUG
    int count = 4;
    #endif
   
    DynamicSample sample;
    
    while(getline(csv_file,line_buffer)) {
        std::stringstream ss(line_buffer);
        
         #ifndef NDEBUG
         std::cout << "DynamicDatasetFile::loadFromFile: Reading " << count << "th line : " << line_buffer << std::endl; 
         count++;
         #endif
    
     
        //Get timestamp
        sample.setTimestamp(get_next_double(ss,data_buffer));
        
        //Get world base orientation (colum major)
        {
            KDL::Rotation world_base_orientation;
            world_base_orientation(0,0) = get_next_double(ss,data_buffer);
            world_base_orientation(1,0) = get_next_double(ss,data_buffer);
            world_base_orientation(2,0) = get_next_double(ss,data_buffer);
            world_base_orientation(0,1) = get_next_double(ss,data_buffer);
            world_base_orientation(1,1) = get_next_double(ss,data_buffer);
            world_base_orientation(2,1) = get_next_double(ss,data_buffer);
            world_base_orientation(0,2) = get_next_double(ss,data_buffer);
            world_base_orientation(1,2) = get_next_double(ss,data_buffer);
            world_base_orientation(2,2) = get_next_double(ss,data_buffer);
            
            KDL::Vector world_base_position;
            world_base_position(0) = get_next_double(ss,data_buffer);
            world_base_position(1) = get_next_double(ss,data_buffer);
            world_base_position(2) = get_next_double(ss,data_buffer);
            
            sample.setWorldBaseTransform(KDL::Frame(world_base_orientation,world_base_position));
        }
        
        //Get base velocity
        {
            KDL::Twist v_base;
            for(int i=0; i < 6; i++ ) {
                v_base[i] = get_next_double(ss,data_buffer);
            }
            sample.getBaseVelocity() = v_base;
        }
        
        //Get base velocity
        {
            KDL::Twist a_base;
            for(int i=0; i < 6; i++ ) {
                a_base[i] = get_next_double(ss,data_buffer);
            }
            sample.setBaseClassicalAcceleration(a_base);
        }
    
        //Get joint positions
        for(int i=0; i < nrOfDOFs; i++ ) {
            sample.setJointPosition(get_next_double(ss,data_buffer),i);
        }
        
        //Get joint speeds
        for(int i=0; i < nrOfDOFs; i++ ) {
            sample.setJointVelocity(get_next_double(ss,data_buffer),i);
        }
        
        //Get joint accelerations
        for(int i=0; i < nrOfDOFs; i++ ) {
            sample.setJointAcceleration(get_next_double(ss,data_buffer),i);
        }
        
        //Get torques measurments
        for(int i=0; i < nrOfMeasuredTorques; i++ ) {
            sample.setTorqueMeasure(get_next_double(ss,data_buffer),i);
        }
        
        //Get wrench measurements
        for(int i=0; i < nrOfMeasuredWrenches; i++) {
            KDL::Wrench f;
            for(int j=0; j < 6; j++ ) {
                f[j] = get_next_double(ss,data_buffer);
            }
            sample.setWrenchMeasure(f,i);
        }
        
        //Get 3 axis force/torque measurements
        for(int i=0; i < nrOfMeasured3AxisFT; i++ ) {
            Eigen::Vector3d three_ft;
            three_ft(0) = get_next_double(ss,data_buffer);
            three_ft(1) = get_next_double(ss,data_buffer);
            three_ft(2) = get_next_double(ss,data_buffer);
            sample.setThreeAxisForceTorqueMeasure(three_ft,i);
        }
        
        //Adding the sample
        dynamic_samples.push_back(sample);
    }
    
    
    
    return true;
}

}