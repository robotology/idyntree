/*
 * Copyright (C) 2013 Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __KDL_CODYCO_REGRESSOR_FIRST_ORDER_NUMERICAL_DIFFERENTIATION_FILE__
#define __KDL_CODYCO_REGRESSOR_FIRST_ORDER_NUMERICAL_DIFFERENTIATION_FILE__

#include <kdl_codyco/regressors/dataset/DynamicDatasetInterfaces.hpp>

namespace KDL::CoDyCo::Regressors {
    
/**
 * Naive calculation of joint velocities and acceleration from joint position
 * 
 */
bool firstOrderNumericalDifferentiation(IBatchDynamicDataset & dataset);




#endif