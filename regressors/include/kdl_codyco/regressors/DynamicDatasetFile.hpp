/*
 * Copyright (C) 2013 Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __KDL_CODYCO_REGRESSOR_DATASET_FILE__
#define __KDL_CODYCO_REGRESSOR_DATASET_FILE__

#include "DynamicDatasetInterfaces.hpp"
#include "DynamicSample.hpp"

namespace KDL {
namespace CoDyCo {
namespace Regressors {

class DynamicDatasetFile : public IBatchDynamicDataset
{
private:
    std::string file_name;
    
    int nrOfDOFs;
    
    int nrOfMeasuredWrenches;
    int nrOfMeasuredTorques;
    int nrOfMeasured3AxisFT;
    
    std::vector<DynamicSample> dynamic_samples;
    
    bool verbose;
    
public:
    DynamicDatasetFile(bool _verbose=true);
    
    bool loadFromFile(std::string filename, const bool append=false);
    
    ~DynamicDatasetFile();
    
    int getNrOfSamples() const;
    
    bool getSample(const int sample_n,DynamicSample & sample) const;
    
    std::string getFileName() const;
};

class DynamicDatasetFileCollection : public std::vector<DynamicDatasetFile>, public IBatchDynamicDataset
{
private:
    int nrOfDOFs;
    
    int nrOfMeasuredWrenches;
    int nrOfMeasuredTorques;
    int nrOfMeasured3AxisFT;
    
    std::vector<int> datasets_samples;
    int nr_of_samples;
public:
    DynamicDatasetFileCollection();
    ~DynamicDatasetFileCollection();
        
    bool loadDatasetFilesFromFilenameVector(const std::vector<std::string> & filenames);
    bool loadDatasetFilesFromFile(const std::string & file_name);
    
    int getNrOfSamples() const;
    
    bool getSample(const int sample_n,DynamicSample & sample) const;
};

}

}
    
}

#endif
