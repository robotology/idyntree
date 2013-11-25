/*
 * Copyright (C) 2013 Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __DIRL_DATASET_FILE__
#define __DIRL_DATASET_FILE__

#include <dirl/dataset/DynamicDatasetInterfaces.hpp>
#include <dirl/dataset/DynamicSample.hpp>

namespace dirl {

class DynamicDatasetFile : public IBatchDynamicDataset
{
private:
    int nrOfDOFs;
    
    int nrOfMeasuredWrenches;
    int nrOfMeasuredTorques;
    int nrOfMeasured3AxisFT;
    
    std::vector<DynamicSample> dynamic_samples;
    
public:
    DynamicDatasetFile();
    
    bool loadFromFile(std::string filename, const bool append=false);
    
    ~DynamicDatasetFile();
    
    int getNrOfSamples();
    
    bool getSample(const int sample_n,DynamicSample & sample);
};

}

#endif