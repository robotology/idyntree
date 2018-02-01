/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef __KDL_CODYCO_REGRESSOR_FIRST_ORDER_NUMERICAL_DIFFERENTIATION_FILE__
#define __KDL_CODYCO_REGRESSOR_FIRST_ORDER_NUMERICAL_DIFFERENTIATION_FILE__

#include "DynamicDatasetInterfaces.hpp"

namespace KDL::CoDyCo::Regressors {
    
/**
 * Naive calculation of joint velocities and acceleration from joint position
 * 
 */
bool firstOrderNumericalDifferentiation(IBatchDynamicDataset & dataset);




#endif