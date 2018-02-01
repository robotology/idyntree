/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <fstream>
#include <iostream>
#include <sstream>

#include "DynamicDatasetFile.hpp"

namespace KDL {
namespace CoDyCo {
namespace Regressors {

DynamicDatasetFileCollection::DynamicDatasetFileCollection()
{
    datasets_samples.resize(0);
    nr_of_samples = 0;
    resize(0);
}

DynamicDatasetFileCollection::~DynamicDatasetFileCollection()
{

}

bool DynamicDatasetFileCollection::loadDatasetFilesFromFilenameVector(const std::vector<std::string> & filenames)
{
    resize(filenames.size());
    datasets_samples.resize(filenames.size());
    for(int i=0; i < filenames.size(); i++ )
    {
        if( ! operator[](i).loadFromFile(filenames[i]) ) {
            return false;
        }
        datasets_samples[i] = operator[](i).getNrOfSamples();
        nr_of_samples += datasets_samples[i];
    }
    return true;
}

bool DynamicDatasetFileCollection::loadDatasetFilesFromFile(const std::string & file_name)
{
     std::vector<std::string> filenames;

     std::ifstream filenames_file;

     filenames_file.open (file_name.c_str(), std::ifstream::in);

     if( !filenames_file ) {
         std::cerr << "DynamicDatasetFileCollection::loadDatasetFilesFromFile error: could not load file " << file_name << std::endl;
         return false;
     }

     std::string data_buffer;
     while(getline(filenames_file,data_buffer)) {
        if( data_buffer == "" ) {
            break;
        }
        filenames.push_back(data_buffer);
     }

     return loadDatasetFilesFromFilenameVector(filenames);

}


int DynamicDatasetFileCollection::getNrOfSamples() const
{
    return nr_of_samples;
}

bool DynamicDatasetFileCollection::getSample(int sample_n, DynamicSample & sample) const
{
    if( sample_n < 0 || sample_n > getNrOfSamples() ) { return false; }
#ifndef NDEBUG
    //std::cerr << " DynamicDatasetFileCollection::("<<sample_n<<",..) returning sample with " << sample.getNrOfDOFs() << " dofs " << std::endl;
#endif
    int start_sample_index = 0;
    for(unsigned int dataset_index=0; dataset_index < size(); dataset_index++ ) {
        if( sample_n-start_sample_index < datasets_samples[dataset_index] ) {
            //Then the requested sample (sample_n) belongs to dataset dataset_index
            return operator[](dataset_index).getSample(sample_n-start_sample_index,sample);
        }

        start_sample_index += datasets_samples[dataset_index];
    }

    return false;
}

}

}

}