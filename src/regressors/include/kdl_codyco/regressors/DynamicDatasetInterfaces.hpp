/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef __KDL_CODYCO_REGRESSOR_DATASET_INTERFACES__
#define __KDL_CODYCO_REGRESSOR_DATASET_INTERFACES__

#include "DynamicSample.hpp"

namespace KDL {
namespace CoDyCo {
namespace Regressors {

/*! \file DynamicDatasetInterfaces.hpp define interfaces for a dynamic identification dataset (online and offline)*/

/**
 *
 * A generic interface to dynamic identification offline dataset
 */
class IBatchDynamicDataset {
public:
    virtual ~IBatchDynamicDataset() {}

    /**
     * It returns the number of samples contained in the dataset
     * @return number of the samples
     */
    virtual int getNrOfSamples() const = 0;

    /**
     * It return the requested sample
     * @param[in] sample_nr the number of the requested sample
     * @param[out] sample the requested sample
     * @return true if all went well, false otherwise
     */
    virtual bool getSample(const int sample_nr, DynamicSample & sample) const = 0;
};

/**
 *
 * A generic interface to dynamic identification online dataset
 */
class IOnlineDynamicDataset {
public:
    virtual ~IOnlineDynamicDataset() {}

    /**
     * It return the new sample (blocking call)
     * @param[out] sample the requested sample
     * @return true if all went well, false otherwise
     */
    virtual bool getNewSample(DynamicSample & sample) = 0;

    /**
     * Check if a new sample is available, or it is necessary to wait
     * @return true if a new sample is available, false if not
     */
    virtual bool isNewSampleAvailable() = 0;
};

}

}

}

#endif
//
