/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
