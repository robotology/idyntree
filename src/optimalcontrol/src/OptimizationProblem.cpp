/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OptimizationProblem.h>
#include <iDynTree/Core/Utils.h>

namespace iDynTree {

    namespace optimization {

        OptimizationProblem::OptimizationProblem()
        :m_infoData(new OptimizationProblemInfoData)
        ,m_info(m_infoData)
        {
        }

        OptimizationProblem::~OptimizationProblem()
        {
        }

        bool OptimizationProblem::prepare()
        {
            reportError("OptimizationProblem", "prepare", "Method not implemented.");
            return false;
        }

        void OptimizationProblem::reset()
        {

        }

        bool OptimizationProblem::getConstraintsBounds(VectorDynSize &constraintsLowerBounds, VectorDynSize &constraintsUpperBounds)
        {
            reportError("OptimizationProblem", "getConstraintsBounds", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getVariablesUpperBound(VectorDynSize &variablesUpperBounds)
        {
            reportError("OptimizationProblem", "getVariablesUpperBounds", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getVariablesLowerBound(VectorDynSize &variablesLowerBound)
        {
            reportError("OptimizationProblem", "getVariablesLowerBound", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getConstraintsJacobianInfo(std::vector<size_t> &nonZeroElementRows, std::vector<size_t> &nonZeroElementColumns)
        {
            reportError("OptimizationProblem", "getConstraintsJacobianInfo", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getHessianInfo(std::vector<size_t> &nonZeroElementRows, std::vector<size_t> &nonZeroElementColumns)
        {
            reportError("OptimizationProblem", "getHessianInfo", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getGuess(VectorDynSize &guess)
        {
            return false;
        }

        bool OptimizationProblem::setVariables(const VectorDynSize &variables)
        {
            reportError("OptimizationProblem", "setVariables", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateCostFunction(double &costValue)
        {
            reportError("OptimizationProblem", "evaluateCostFunction", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateCostGradient(VectorDynSize &gradient)
        {
            reportError("OptimizationProblem", "evaluateCostGradient", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateCostHessian(MatrixDynSize &hessian)
        {
            reportError("OptimizationProblem", "evaluateCostHessian", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateConstraints(VectorDynSize &constraints)
        {
            reportError("OptimizationProblem", "evaluateConstraints", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateConstraintsJacobian(MatrixDynSize &jacobian)
        {
            reportError("OptimizationProblem", "evaluateConstraintsJacobian", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateConstraintsHessian(const VectorDynSize &constraintsMultipliers, MatrixDynSize &hessian)
        {
            reportError("OptimizationProblem", "evaluateConstraintsHessian", "Method not implemented.");
            return false;
        }

        const OptimizationProblemInfo &OptimizationProblem::info() const
        {
            return m_info;
        }

        OptimizationProblemInfoData::OptimizationProblemInfoData()
        : hasLinearConstraints(false)
        , hasNonLinearConstraints(true)
        , costIsLinear(false)
        , costIsQuadratic(false)
        , costIsNonLinear(true)
        , hasSparseConstraintJacobian(false)
        , hasSparseHessian(false)
        , hessianIsProvided(false)
        { }

        OptimizationProblemInfo::OptimizationProblemInfo(std::shared_ptr<OptimizationProblemInfoData> data)
            : m_data(data)
        { }

        bool OptimizationProblemInfo::hasLinearConstraints() const
        {
            return m_data->hasLinearConstraints;
        }

        bool OptimizationProblemInfo::hasNonLinearConstraints() const
        {
            return m_data->hasNonLinearConstraints;
        }

        bool OptimizationProblemInfo::costIsLinear() const
        {
            return m_data->costIsLinear;
        }

        bool OptimizationProblemInfo::costIsQuadratic() const
        {
            return m_data->costIsQuadratic;
        }

        bool OptimizationProblemInfo::costIsNonLinear() const
        {
            return m_data->costIsNonLinear;
        }

        bool OptimizationProblemInfo::hasSparseConstraintJacobian() const
        {
            return m_data->hasSparseConstraintJacobian;
        }

        bool OptimizationProblemInfo::hasSparseHessian() const
        {
            return m_data->hasSparseHessian;
        }

        bool OptimizationProblemInfo::hessianIsProvided() const
        {
            return m_data->hessianIsProvided;
        }

    }
}
