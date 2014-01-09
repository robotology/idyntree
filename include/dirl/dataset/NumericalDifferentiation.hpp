/*
 * Copyright (C) 2013 Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __DIRL_FIRST_ORDER_NUMERICAL_DIFFERENTIATION_FILE__
#define __DIRL_FIRST_ORDER_NUMERICAL_DIFFERENTIATION_FILE__

#include <dirl/dataset/DynamicDatasetInterfaces.hpp>

namespace dirl {
    
/**
 * Naive calculation of joint velocities and acceleration from joint position
 * 
 */
bool firstOrderNumericalDifferentiation(IBatchDynamicDataset & dataset);




#endif