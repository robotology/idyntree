/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include "iDynTree/Cost.h"
#include "iDynTree/Core/Utils.h"
#include <sstream>

namespace iDynTree {
    namespace optimalcontrol{
        Cost::Cost(const std::string &costName)
            :m_costName(costName)
        { }

        Cost::~Cost()
        { }

        const std::string Cost::name() const
        {
            return m_costName;
        }

        bool Cost::costFirstPartialDerivativeWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &partialDerivative)
        {
            std::ostringstream errorMsg;
            errorMsg << "Method not implemented for cost "<< name() << std::endl;
            reportError("Cost", "costFirstPartialDerivativeWRTState", errorMsg.str().c_str());
            return false;
        }

        bool Cost::costFirstPartialDerivativeWRTControl(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &partialDerivative)
        {
            std::ostringstream errorMsg;
            errorMsg << "Method not implemented for cost "<< name() << std::endl;
            reportError("Cost", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
            return false;
        }

        bool Cost::costSecondPartialDerivativeWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            std::ostringstream errorMsg;
            errorMsg << "Method not implemented for cost "<< name() << std::endl;
            reportError("Cost", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
            return false;
        }

        bool Cost::costSecondPartialDerivativeWRTControl(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            std::ostringstream errorMsg;
            errorMsg << "Method not implemented for cost "<< name() << std::endl;
            reportError("Cost", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
            return false;
        }

        bool Cost::costSecondPartialDerivativeWRTStateControl(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            std::ostringstream errorMsg;
            errorMsg << "Method not implemented for cost "<< name() << std::endl;
            reportError("Cost", "costSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
            return false;
        }

    }

}
