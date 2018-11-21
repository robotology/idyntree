/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
//For using the M_PI macro in visual studio it
//is necessary to define _USE_MATH_DEFINES
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "InverseKinematicsNLP.h"

#include "InverseKinematicsData.h"
#include "TransformConstraint.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <cassert>
#include <cmath>


namespace internal {
namespace kinematics {

    // Check of MatrixFixSize
    template<typename>
    struct is_matrixfixsize : std::false_type {};

    template<unsigned row, unsigned col>
    struct is_matrixfixsize<iDynTree::MatrixFixSize<row, col>> : std::true_type {};

    //MARK: - SparsityHelper implementation

    const std::vector<size_t> SparsityHelper::s_nullVector = std::vector<size_t>();
    const std::vector<size_t>& SparsityHelper::NullIndicesVector() { return s_nullVector; }

    SparsityHelper::SparsityHelper()
    {
        m_numberOfNonZeros.resize(1, 0);
    }

    void SparsityHelper::clear()
    {
        m_nonZeroIndices.clear();
        m_numberOfNonZeros.clear();
        m_numberOfNonZeros.resize(1, 0);
    }

    template <typename MatrixType>
    bool SparsityHelper::addConstraintSparsityPatternTemplated(const MatrixType& newConstraint, const iDynTree::IndexRange& constraintRange)
    {
        static_assert(std::is_base_of<MatrixType, iDynTree::MatrixDynSize>::value
                      || is_matrixfixsize<MatrixType>::value,
                      "addConstraintSparsityPatternTemplated can be called only with iDynTree::MatrixFixSize and iDynTree::MatrixDynSize");

        size_t constraintSize = constraintRange.size;
        if (constraintSize == 0) return false;
        size_t previousConstraintSize = m_nonZeroIndices.size();

        // append this constraint to the current pattern
        // Update the number of nonzeros
        m_numberOfNonZeros.resize(1 + previousConstraintSize + constraintSize, m_numberOfNonZeros[previousConstraintSize]);
        m_nonZeroIndices.resize(previousConstraintSize + constraintSize);
        for (size_t row = 0; row < constraintSize; ++row) {
            size_t numberOfNonZeros = static_cast<size_t>(iDynTree::toEigen(newConstraint).row(constraintRange.offset + row).sum());
            m_numberOfNonZeros[1 + previousConstraintSize + row] = totalNumberOfNonZerosBeforeRow(previousConstraintSize + row) +  numberOfNonZeros;
            assert(numberOfNonZeros == numberOfNonZerosForRow(previousConstraintSize + row));

            // Update indices
            m_nonZeroIndices[previousConstraintSize + row].reserve(numberOfNonZeros);
            for (size_t col = 0; col < newConstraint.cols(); ++col) {
                if (std::abs(newConstraint(constraintRange.offset + row, col) - 1) < iDynTree::DEFAULT_TOL) {
                    m_nonZeroIndices[previousConstraintSize + row].push_back(col);
                }
            }
        }

        return true;
    }


    bool SparsityHelper::addConstraintSparsityPattern(const iDynTree::MatrixDynSize& newConstraint)
    {
        return addConstraintSparsityPattern(newConstraint, {0, static_cast<ptrdiff_t>(newConstraint.rows())});
    }

    template<unsigned int nRows, unsigned int nCols>
    bool SparsityHelper::addConstraintSparsityPattern(const iDynTree::MatrixFixSize<nRows, nCols>& newConstraint)
    {
        return addConstraintSparsityPattern(newConstraint, {0, static_cast<ptrdiff_t>(newConstraint.rows())});
    }

    bool SparsityHelper::addConstraintSparsityPattern(const iDynTree::MatrixDynSize& newConstraint,
                                                      const iDynTree::IndexRange& constraintRange)
    {
        return addConstraintSparsityPatternTemplated(newConstraint, constraintRange);
    }

    template<unsigned int nRows, unsigned int nCols>
    bool SparsityHelper::addConstraintSparsityPattern(const iDynTree::MatrixFixSize<nRows, nCols>& newConstraint,
                                                      const iDynTree::IndexRange& constraintRange)
    {
        return addConstraintSparsityPatternTemplated(newConstraint, constraintRange);
    }



    size_t SparsityHelper::numberOfNonZerosForRow(size_t rowIndex) const
    {
        if (rowIndex >= m_numberOfNonZeros.size() - 1) return 0;
        return m_numberOfNonZeros[rowIndex + 1] - m_numberOfNonZeros[rowIndex];
    }
    size_t SparsityHelper::numberOfNonZeros() const { return m_numberOfNonZeros[m_numberOfNonZeros.size() - 1]; }
    size_t SparsityHelper::totalNumberOfNonZerosBeforeRow(size_t rowIndex) const
    {
        if (rowIndex >= m_numberOfNonZeros.size() - 1) return 0;
        return m_numberOfNonZeros[rowIndex];
    }

    const std::vector<size_t>& SparsityHelper::nonZeroIndicesForRow(size_t rowIndex) const
    {
        if (rowIndex > m_nonZeroIndices.size()) return s_nullVector;
        return m_nonZeroIndices[rowIndex];
    }

    void SparsityHelper::assignActualMatrixValues(const iDynTree::IndexRange& constraintRange,
                                                  const iDynTree::MatrixDynSize& fullMatrix,
                                                  size_t fullMatrixStartingRowIndex,
                                                  Ipopt::Number *outputBuffer)
    {
        for (size_t row = 0; row < constraintRange.size; ++row) {
            // get the current row in the sparsity pattern matrix
            const std::vector<size_t>& currentSparsityRow = nonZeroIndicesForRow(constraintRange.offset + row);
            Ipopt::Number *startingValue = &outputBuffer[totalNumberOfNonZerosBeforeRow(constraintRange.offset + row)];

            for (size_t colIndex = 0; colIndex < currentSparsityRow.size(); ++colIndex) {
                startingValue[colIndex] = fullMatrix(fullMatrixStartingRowIndex + row, currentSparsityRow[colIndex]);
            }
        }
    }

    std::string SparsityHelper::toString() const
    {
        size_t maxCol = 0;
        for (const auto& row : m_nonZeroIndices) {
            if (row.empty()) continue;
            maxCol = std::max(maxCol, 1 + *(--row.end()));
        }

        iDynTree::MatrixDynSize matrix(m_nonZeroIndices.size(), maxCol);
        matrix.zero();
        if (maxCol > 0) {
            for (size_t row = 0; row < m_nonZeroIndices.size(); ++row) {
                for (const auto& col : m_nonZeroIndices[row]) {
                    matrix(row, col) = 1;
                }
            }
        }
        return matrix.toString();
    }

    //MARK: - InverseKinematicsNLP implementation

#ifndef NDEBUG
    bool InverseKinematicsNLP::eval_f_called = false;
    bool InverseKinematicsNLP::eval_grad_f_called = false;
    bool InverseKinematicsNLP::eval_g_called = false;
    bool InverseKinematicsNLP::eval_jac_g_called = false;
#endif

    //To understand IPOpt log:
    //http://www.coin-or.org/Ipopt/documentation/node36.html#sec:output

    InverseKinematicsNLP::InverseKinematicsNLP(InverseKinematicsData& data)
    : m_data(data)
    { }

    InverseKinematicsNLP::~InverseKinematicsNLP() {}


    void InverseKinematicsNLP::initializeInternalData()
    {
        //This method should initialize all the internal buffers used during
        //the optimization
        optimizedBaseOrientation.zero();
        jointsAtOptimisationStep.resize(m_data.m_dofs);
        // initialize this with the current state of dyn computations
        m_data.m_dynamics.getJointPos(jointsAtOptimisationStep);

        //resize some buffers
        finalJacobianBuffer.resize((3 + sizeOfRotationParametrization(m_data.m_rotationParametrization)),
                                   3 + sizeOfRotationParametrization(m_data.m_rotationParametrization) + m_data.m_dofs);

        constraintsInfo.clear();
        targetsInfo.clear();

        //prepare buffers for constraints and targets
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
             target != m_data.m_targets.end(); ++target) {
            FrameInfo info;
            info.jacobian.resize(6, m_data.m_dofs + 6);
            info.jacobian.zero();
            targetsInfo.insert(FrameInfoMap::value_type(target->first, info));
        }

        for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
             constraint != m_data.m_constraints.end(); ++constraint) {
            FrameInfo info;
            info.jacobian.resize(6, m_data.m_dofs + 6);
            info.jacobian.zero();
            constraintsInfo.insert(FrameInfoMap::value_type(constraint->first, info));
        }

        //prepare buffer for COM constraint
        comInfo.com.zero();
        comInfo.comJacobian.resize(3, m_data.m_dofs + 6);
        comInfo.comJacobianAnalytical.resize(3, m_data.m_dofs + 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization));
        comInfo.projectedComJacobian.resize(m_data.m_comHullConstraint.getNrOfConstraints(), m_data.m_dofs + 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization));

        initializeSparsityInformation();
    }

void InverseKinematicsNLP::addSparsityInformationForConstraint(int constraintID,
                                                               const internal::kinematics::TransformConstraint& constraint)
    {
        //For each constraint compute its jacobian pattern
        FrameInfo &constraintInfo = constraintsInfo[constraintID];
        // iDynTree pattern
        m_data.dynamics().getFrameFreeFloatingJacobianSparsityPattern(constraintID, constraintInfo.jacobian);
        // Now we have to modify it depending on the orientation parametrization


        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

            // Create sparsity of quaternion inverse and direct maps
            // Does not have any sparsity.
            iDynTree::MatrixFixSize<3, 4> quaternionDerivativeInverseMap;
            iDynTree::toEigen(quaternionDerivativeInverseMap).setOnes();
            iDynTree::MatrixFixSize<4, 3> quaternionDerivativeMap;
            iDynTree::toEigen(quaternionDerivativeMap).setOnes();

            finalJacobianBuffer.zero();
            computeConstraintJacobian(constraintInfo.jacobian, // this has the sparsity
                                      quaternionDerivativeMap, // this is a all 1s matrix
                                      quaternionDerivativeInverseMap, // this is a all 1s matrix
                                      ComputeContraintJacobianOptionLinearPart|ComputeContraintJacobianOptionAngularPart,
                                      finalJacobianBuffer);


        } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
            //RPY sparsity parametrization for the base
            /*  -       -
             * | x  x  0 |
             * | x  x  0 |
             * | x  0  x |
             *  -       -
             */
            iDynTree::Matrix3x3 RPYToOmega;
            iDynTree::toEigen(RPYToOmega).setIdentity();
            iDynTree::toEigen(RPYToOmega).topLeftCorner<2, 2>().setOnes();
            RPYToOmega(2, 0) = 1.0;

            // RPY sparsity parametrization for the constraint
            /*  -       -
             * | x  x  0 |
             * | x  x  0 |
             * | x  x  x |
             *  -       -
             */
            iDynTree::Matrix3x3 omegaToRPYMap_target;
            iDynTree::toEigen(omegaToRPYMap_target).setZero();
            iDynTree::toEigen(omegaToRPYMap_target).topLeftCorner<2, 2>().setOnes();
            iDynTree::toEigen(omegaToRPYMap_target).bottomRows<1>().setOnes();


            computeConstraintJacobianRPY(constraintInfo.jacobian,
                                         omegaToRPYMap_target,
                                         RPYToOmega,
                                         ComputeContraintJacobianOptionLinearPart|ComputeContraintJacobianOptionAngularPart,
                                         finalJacobianBuffer);
        }

        // Now "normalize" (i.e. only 0.0 and 1.0) the result
        for (unsigned row = 0; row < finalJacobianBuffer.rows(); ++row) {
            for (unsigned col = 0; col < finalJacobianBuffer.cols(); ++col) {
                finalJacobianBuffer(row, col) = std::abs(finalJacobianBuffer(row, col)) < iDynTree::DEFAULT_TOL ? 0.0 : 1.0;
            }
        }

        //Now that we computed the actual Jacobian needed by IPOPT
        //We have to assign it to the correct variable
        if (constraint.hasPositionConstraint()) {
            //Position part
            m_jacobianSparsityHelper.addConstraintSparsityPattern(finalJacobianBuffer, {0, 3});
        }
        if (constraint.hasRotationConstraint()) {
            //Orientation part
            m_jacobianSparsityHelper.addConstraintSparsityPattern(finalJacobianBuffer, {3, sizeOfRotationParametrization(m_data.m_rotationParametrization)});
        }
    }

    void InverseKinematicsNLP::initializeSparsityInformation()
    {
        m_jacobianSparsityHelper.clear();

        for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
             constraint != m_data.m_constraints.end(); ++constraint) {
            if (constraint->second.isActive()) {
                addSparsityInformationForConstraint(constraint->first, constraint->second);
            }
        }

        //For COM constraint
        //RPY parametrization for the base
        if (m_data.m_comHullConstraint.isActive() || (m_data.isCoMTargetActive() && m_data.isCoMaConstraint())) {
            //TODO: implement quaternion part
            assert(m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

            // CoM Jacobian is always full by definition
            iDynTree::toEigen(comInfo.comJacobianAnalytical).setOnes();

            if (m_data.m_comHullConstraint.isActive()) {
                iDynTree::toEigen(comInfo.projectedComJacobian).setOnes();
                m_jacobianSparsityHelper.addConstraintSparsityPattern(comInfo.projectedComJacobian);
            }

            if (m_data.isCoMTargetActive()) {
                m_jacobianSparsityHelper.addConstraintSparsityPattern(comInfo.comJacobianAnalytical);
            }
        }


        //For all targets enforced as constraints
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
             target != m_data.m_targets.end(); ++target) {

            if (target->second.isActive()) {
                //Depending if we need position and/or orientation
                //we have to adapt different parts of the jacobian
                int computationOption = 0;
                if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly)
                    computationOption |= ComputeContraintJacobianOptionLinearPart;
                if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly)
                    computationOption |= ComputeContraintJacobianOptionAngularPart;

                if (computationOption == 0) continue; // no need for further computations

                addSparsityInformationForConstraint(target->first, target->second);
            }

        }


        //Finally, the norm of the base orientation quaternion parametrization
        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            iDynTree::MatrixFixSize<1, 7> baseQuaternionConstraint;
            baseQuaternionConstraint.zero();
            iDynTree::toEigen(baseQuaternionConstraint).rightCols<4>().setOnes();
            m_jacobianSparsityHelper.addConstraintSparsityPattern(baseQuaternionConstraint);
        }



    }

    bool InverseKinematicsNLP::updateState(const Ipopt::Number * x)
    {
        //This method computes all the data which is needed in more than one place.

        //first: save robot configuration
        //position
        optimizedBasePosition(0) = x[0];
        optimizedBasePosition(1) = x[1];
        optimizedBasePosition(2) = x[2];

        //orientation
        for (unsigned i = 0; i < sizeOfRotationParametrization(m_data.m_rotationParametrization); ++i) {
            this->optimizedBaseOrientation(i) = x[3 + i];
        }

        //joints
        Eigen::Map<const Eigen::VectorXd> optimisedJoints(&x[3 + sizeOfRotationParametrization(m_data.m_rotationParametrization)], m_data.m_dofs);
        iDynTree::toEigen(jointsAtOptimisationStep) = optimisedJoints;

        //Update robot state and state-dependent variables
        iDynTree::Rotation baseOrientationRotation;

        //Get the rotation from the used serialization
        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            //Quaternion parametrization
            baseOrientationRotation.fromQuaternion(this->optimizedBaseOrientation);

            /*! As we used a quaternion we have to update the maps
             * used when we compute the derivatives as we
             * need to update the iDynTree Jacobians.
             * See Eq. 16 of the IK document
             * i.e.
             *
             * \f[ J \Rightarrow \bar{J} \f]
             * that is
             *
             */
            iDynTree::Vector4 normQuaternioniDyn = this->optimizedBaseOrientation;
            iDynTree::toEigen(normQuaternioniDyn).normalize();

            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > inverseMap = iDynTree::toEigen(quaternionDerivativeInverseMapBuffer);

            //Set the inverse map
            /*!
             * Inverse map: from omega to derivative of the quaternion
             * This is done only once as it depends on the quaternion parametrization
             * of the orientation of the base
             *
             * \f[
             * G^{-1}(z) = 2 \begin{bmatrix}
             * -r & -r^\wedge + s 1_3
             * \end{bmatrix}.
             * \f]
             */
            inverseMap.setZero();
            inverseMap.leftCols<1>() = -iDynTree::toEigen(normQuaternioniDyn).tail<3>();
            inverseMap.rightCols<3>().setIdentity();
            inverseMap.rightCols<3>() *= iDynTree::toEigen(normQuaternioniDyn)(0);
            inverseMap.rightCols<3>() += iDynTree::skew(iDynTree::toEigen(normQuaternioniDyn).tail<3>());

            inverseMap *= 2;

            /*!
             * We also have to add an additional component to the inverse map
             * It can happen that the quaternion given by IPOPT is not normalized.
             * This matrix corrects this possibility by adding the derivative of a unit quaternion
             * w.r.t. a non-unitary quaternion, i.e.
             * \f[
             *  \frac{\partial z_f}{\partial \bar{z}_B} &= \frac{\partial z_f}{\partial z_B} \frac{\partial z_B}{\partial \bar{z}_B}
             * \f]
             * where \f$ \bar{z}_B : z_B = \frac{\bar{z}_B}{\norm{\bar{z}_B}} \f$
             */
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> normalizedQuaternionDerivative;
            Eigen::Map<Eigen::Vector4d> quaternion = iDynTree::toEigen(this->optimizedBaseOrientation);
            double quaternionSNorm = quaternion.squaredNorm();
            normalizedQuaternionDerivative.setIdentity();
            normalizedQuaternionDerivative *= quaternionSNorm;
            normalizedQuaternionDerivative -= (quaternion * quaternion.transpose());
            normalizedQuaternionDerivative /= std::pow(quaternionSNorm, 3.0/2.0);

            inverseMap *= normalizedQuaternionDerivative;

        } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
            //RPY parametrization
            baseOrientationRotation = iDynTree::Rotation::RPY(this->optimizedBaseOrientation(0), this->optimizedBaseOrientation(1), this->optimizedBaseOrientation(2));
        }
        // base pose
        iDynTree::Transform basePose(baseOrientationRotation, optimizedBasePosition);

        if (!m_data.m_dynamics.setRobotState(basePose,
                                             jointsAtOptimisationStep,
                                             m_data.m_state.baseTwist,
                                             m_data.m_state.jointsVelocity,
                                             m_data.m_state.worldGravity)) {
            return false;
        }

        // Common computation
        // - for each target: transform (f, grad_f, g, grad_g)
        // - Jacobian of target frames (grad_f, grad_g)
        // - transform of constraint frame (g, grad_g)
        // - Jacobian of constraint frame (grad_g)
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
             target != m_data.m_targets.end(); ++target) {

            FrameInfo &frameInfo = targetsInfo[target->first];
            frameInfo.transform = m_data.m_dynamics.getWorldTransform(target->first);
            m_data.m_dynamics.getFrameFreeFloatingJacobian(target->first, frameInfo.jacobian);

            iDynTree::Vector4 transformQuat;
            frameInfo.transform.getRotation().getQuaternion(transformQuat);
            //compute quaternionDerivativeMapBuffer
            /*!
             * Direct map: from omega to quaternion derivative to omega
             * As this depends on the frame we have to compute it for each frame
             *
             * \f[
             * G(z) = \frac{1}{2} \begin{bmatrix}
             * -r^\top \\
             * r^\wedge + s 1_3
             * \end{bmatrix}.
             * \f]
             */

            Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > map = iDynTree::toEigen(frameInfo.quaternionDerivativeMap);
            map.topRows<1>() = -iDynTree::toEigen(transformQuat).tail<3>().transpose();
            map.bottomRows<3>().setIdentity();
            map.bottomRows<3>() *= transformQuat(0);
            map.bottomRows<3>() -= iDynTree::skew(iDynTree::toEigen(transformQuat).tail<3>());

            map *= 0.5;
        }

        for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
             constraint != m_data.m_constraints.end(); ++constraint) {

            if (constraint->second.isActive()) {
                FrameInfo &frameInfo = constraintsInfo[constraint->first];
                frameInfo.transform = m_data.m_dynamics.getWorldTransform(constraint->first);
                m_data.m_dynamics.getFrameFreeFloatingJacobian(constraint->first, frameInfo.jacobian);

                iDynTree::Vector4 transformQuat;
                frameInfo.transform.getRotation().getQuaternion(transformQuat);
                //compute quaternionDerivativeMapBuffer
                //See in target for details on the computation
                Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > map = iDynTree::toEigen(
                        frameInfo.quaternionDerivativeMap);
                map.topRows<1>() = -iDynTree::toEigen(transformQuat).tail<3>().transpose();
                map.bottomRows<3>().setIdentity();
                map.bottomRows<3>() *= transformQuat(0);
                map.bottomRows<3>() -= iDynTree::skew(iDynTree::toEigen(transformQuat).tail<3>());

                map *= 0.5;
            }
        }

        // Update com position and jacobian
        if (m_data.m_comHullConstraint.isActive() || m_data.isCoMTargetActive()) {
            comInfo.com = m_data.m_dynamics.getCenterOfMassPosition();
            m_data.m_dynamics.getCenterOfMassJacobian(comInfo.comJacobian);

            if (m_data.m_comHullConstraint.isActive()) {
                // Project the COM along the desired direction
                comInfo.projectedCom = m_data.m_comHullConstraint.projectAlongDirection(comInfo.com);
            }
        }

        return true;
    }

    bool InverseKinematicsNLP::get_nlp_info(Ipopt::Index& n,
                                            Ipopt::Index& m,
                                            Ipopt::Index& nnz_jac_g,
                                            Ipopt::Index& nnz_h_lag,
                                            IndexStyleEnum& index_style)
    {
        n = m_data.m_numberOfOptimisationVariables;
        m = m_data.m_numberOfOptimisationConstraints;

        nnz_jac_g = m_jacobianSparsityHelper.numberOfNonZeros();

        nnz_h_lag = n * n; //this is currently ignored

        index_style = C_STYLE;

        return true;
    }

    bool InverseKinematicsNLP::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)
    {
        Ipopt::Index baseSize = 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization);

        // Bound constraints
        //no constraints on the origin of the base
        for (Ipopt::Index i = 0; i < 3; ++i) {
            x_l[i] = -2e+19;
            x_u[i] =  2e+19;
        }

        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            //Quaternion: limits on the elements
            x_l[3] = 0; x_u[3] = 1;
            x_l[4] = -1; x_u[4] = 1;
            x_l[5] = -1; x_u[5] = 1;
            x_l[6] = -1; x_u[6] = 1;
        } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
            //RPY: limit the rotation
            x_l[3] = -M_PI;
            x_u[3] =  M_PI;
            x_l[4] = -M_PI;
            x_u[4] =  M_PI;
            x_l[5] = -M_PI;
            x_u[5] =  M_PI;
        }

        //for joints use the limits
        for (Ipopt::Index i = baseSize; i < n; ++i) {
            if (m_data.m_reducedVariablesInfo.fixedVariables[i - baseSize]) {
                x_l[i] = x_u[i] = m_data.m_jointInitialConditions(i - baseSize);
            } else {
                const std::pair<double, double> &limits = m_data.m_jointLimits[i - baseSize];
                x_l[i] = limits.first;
                x_u[i] = limits.second;
            }
        }

        //Equality constraints
        Ipopt::Index constraintIndex = 0;
        for (TransformMap::const_iterator it = m_data.m_constraints.begin();
             it != m_data.m_constraints.end(); it++) {
            if (it->second.isActive()) {
                //This is a constraint on a frame position
                if (it->second.hasPositionConstraint()) {
                    const iDynTree::Position &position = it->second.getPosition();
                    g_l[constraintIndex] = g_u[constraintIndex] = position(0);
                    constraintIndex++;
                    g_l[constraintIndex] = g_u[constraintIndex] = position(1);
                    constraintIndex++;
                    g_l[constraintIndex] = g_u[constraintIndex] = position(2);
                    constraintIndex++;
                }
                //This is a constraint on a frame orientation
                if (it->second.hasRotationConstraint()) {
                    const iDynTree::Rotation &rotation = it->second.getRotation();

                    //Get the desired orientation in its representation
                    if (m_data.m_rotationParametrization ==
                        iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                        iDynTree::Vector4 quaternion;
                        rotation.getQuaternion(quaternion);

                        for (Ipopt::Index i = 0; i < 4; ++i)
                        {
                            g_l[constraintIndex] = g_u[constraintIndex] = quaternion(i);
                            constraintIndex++;
                        }

                    } else if (m_data.m_rotationParametrization ==
                               iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
                        iDynTree::Vector3 rpy;
                        rotation.getRPY(rpy(0), rpy(1), rpy(2));

                        for (Ipopt::Index i = 0; i < 3; ++i)
                        {
                            g_l[constraintIndex] = g_u[constraintIndex] = rpy(i);
                            constraintIndex++;
                        }
                    }
                }
            }
        }

        // COM Convex Hull constraint (Ax <= b)
        if (m_data.m_comHullConstraint.isActive()) {
            for (int i = 0; i < m_data.m_comHullConstraint.getNrOfConstraints(); i++) {
                g_l[constraintIndex] = -2e19;
                g_u[constraintIndex] = m_data.m_comHullConstraint.b(i);
                constraintIndex++;
            }
        }
        
        //COM target treated as constraint
        if (m_data.isCoMTargetActive() && m_data.isCoMaConstraint()){
           for (int i = 0; i<3; ++i){
               g_l[constraintIndex] = - m_data.m_comTarget.constraintTolerance;
               g_u[constraintIndex] =   m_data.m_comTarget.constraintTolerance;
               constraintIndex++;
           }
        }


        //target <=> position constraint
        for (TransformMap::const_iterator it = m_data.m_targets.begin();
             it != m_data.m_targets.end(); ++it) {
            if (it->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                && it->second.hasPositionConstraint()) {
                //This target position is considered as constraint
                const iDynTree::Position& position = it->second.getPosition();
                g_l[constraintIndex] = g_u[constraintIndex] = position(0);
                constraintIndex++;
                g_l[constraintIndex] = g_u[constraintIndex] = position(1);
                constraintIndex++;
                g_l[constraintIndex] = g_u[constraintIndex] = position(2);
                constraintIndex++;
            }
            if (it->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                && it->second.hasRotationConstraint()) {
                //This target orientation is considered as a constraint
                const iDynTree::Rotation& rotation = it->second.getRotation();

                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                    iDynTree::Vector4 quaternion;
                    rotation.getQuaternion(quaternion);

                    for (Ipopt::Index i = 0; i < 4; ++i) {
                        g_l[constraintIndex] = g_u[constraintIndex] = quaternion(i);
                        constraintIndex++;
                    }

                } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
                    iDynTree::Vector3 rpy;
                    rotation.getRPY(rpy(0), rpy(1), rpy(2));

                    for (Ipopt::Index i = 0; i < 3; ++i) {
                        g_l[constraintIndex] = g_u[constraintIndex] = rpy(i);
                        constraintIndex++;
                    }
                }
            }
        }

        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            //quaternion unit norm constraint
            // || Q(q) ||^2 = 1
            g_l[constraintIndex] = g_u[constraintIndex] = 1;
            constraintIndex++;
        }

        assert(constraintIndex == m);
        return true;
    }

    bool InverseKinematicsNLP::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                                  Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda)
    {
        UNUSED_VARIABLE(z_L);
        UNUSED_VARIABLE(z_U);
        UNUSED_VARIABLE(m);
        UNUSED_VARIABLE(lambda);

        //initial conditions
        if (init_x) {
            iDynTree::Position position = m_data.m_baseInitialCondition.getPosition();
            for (Ipopt::Index i = 0; i < 3; ++i) {
                x[i] = position(i);
            }

            Eigen::Map<Eigen::VectorXd> baseRotation(&x[3], sizeOfRotationParametrization(m_data.m_rotationParametrization));

            switch (m_data.m_rotationParametrization) {
                case iDynTree::InverseKinematicsRotationParametrizationQuaternion:
                    baseRotation = iDynTree::toEigen(m_data.m_baseInitialCondition.getRotation().asQuaternion());
                    break;
                case iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw:
                    baseRotation.head<3>() = iDynTree::toEigen(m_data.m_baseInitialCondition.getRotation().asRPY());
                    break;
                default:
                    return false;
            }

            Ipopt::Index baseSize = 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization);
            Eigen::Map<Eigen::VectorXd> joints(&x[baseSize], n - baseSize);
            joints = iDynTree::toEigen(m_data.m_jointInitialConditions);

        }

        if (init_z) {
            Eigen::Map<Eigen::VectorXd> lowerBounds(z_L, n);
            Eigen::Map<Eigen::VectorXd> upperBounds(z_U, n);

            lowerBounds = iDynTree::toEigen(m_data.m_lowerBoundMultipliers);
            upperBounds = iDynTree::toEigen(m_data.m_upperBoundMultipliers);
        }
        if (init_lambda) {
            Eigen::Map<Eigen::VectorXd> multipliers(lambda, m);

            multipliers = iDynTree::toEigen(m_data.m_constraintMultipliers);
        }
        return true;
    }

    bool InverseKinematicsNLP::eval_f(Ipopt::Index n, const Ipopt::Number* x,
                                      bool new_x, Ipopt::Number& obj_value)
    {
        UNUSED_VARIABLE(n);
        if (new_x) {
#ifndef NDEBUG
            eval_f_called = false;
            eval_grad_f_called = false;
            eval_g_called = false;
            eval_jac_g_called = false;
#endif
            //First time we get called with this new value for the solution
            //Update the state and variables
            if (!updateState(x))
                return false;
        }
#ifndef NDEBUG
        assert(!eval_f_called);
        eval_f_called = true;
#endif

        //Cost function
        //J = Sum_i^#targets  Error on rotation   + ||q - q_des ||^2 (as regularization term)

        Eigen::VectorXd jointError = iDynTree::toEigen(jointsAtOptimisationStep)-iDynTree::toEigen(m_data.m_preferredJointsConfiguration);
        const iDynTree::VectorDynSize& jointCostWeight = m_data.m_preferredJointsWeight;

        obj_value = 0;
        for (size_t i = 0; i < jointError.size(); ++i) {
            obj_value += 0.5 * jointCostWeight(i) * jointError(i) * jointError(i);
        }

        //compute errors on rotation
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
                target != m_data.m_targets.end(); ++target) {

            if ((target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly ||
                    target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintNone)
                && target->second.hasPositionConstraint()) {
                //this implies that position is a soft constraint.
                iDynTree::Position positionError = targetsInfo[target->first].transform.getPosition() - target->second.getPosition();
                obj_value += 0.5 * target->second.getPositionWeight() * (iDynTree::toEigen(positionError)).squaredNorm();
            }
            if ((target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly ||
                    target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintNone)
                && target->second.hasRotationConstraint()) {

                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion){

                    //this implies that rotation is a soft constraint.
                    //Get actual and desired orientation of target and compute the
                    //orientation error, as  w_R_f * (w_R_f^d)^{-1} = w_\tilde{R}_w (R tilde expressed in inertial)
                    //TODO: check if it is correct to express it in the inertial
                    iDynTree::Rotation transformError = targetsInfo[target->first].transform.getRotation() * target->second.getRotation().inverse();

                    //Quaternion corresponding to the orientation error
                    iDynTree::Vector4 orientationErrorQuaternion;
                    transformError.getQuaternion(orientationErrorQuaternion);

                    iDynTree::Vector4 identityQuaternion;
                    iDynTree::Rotation::Identity().getQuaternion(identityQuaternion);

                    //Implementing orientation cost as
                    // \tilde(Q) = Q_error
                    // Q_1 = Q_identity
                    //|| \tilde(Q) - Q_1 ||^2 as measure of error
                    // there is an alternative cost. See latex (using trace of R)
                    obj_value += 0.5 * target->second.getRotationWeight() * (iDynTree::toEigen(orientationErrorQuaternion) - iDynTree::toEigen(identityQuaternion)).squaredNorm();
                }
                else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw){
                    FrameInfo &targetInfo = targetsInfo[target->first];

                    iDynTree::Rotation rotation_target = targetInfo.transform.getRotation();
                    iDynTree::Rotation rotation_desired = target->second.getRotation();
                    iDynTree::Rotation rotation_error;
                    iDynTree::toEigen(rotation_error) = iDynTree::toEigen(rotation_desired).transpose() * iDynTree::toEigen(rotation_target);
                    
                    //TODO Investigate the derivative of the cost using the RPY representation of the error matrix R*\hat{R}'
                    obj_value += 0.5 * target->second.getRotationWeight() * iDynTree::toEigen(rotation_error.asRPY()).squaredNorm();
                }
            }
        }


        if (m_data.isCoMTargetActive() && !(m_data.isCoMaConstraint())){
            iDynTree::Position comPositionError;
            comPositionError = comInfo.com - m_data.m_comTarget.desiredPosition;
            obj_value += 0.5 * m_data.m_comTarget.weight * iDynTree::toEigen(comPositionError).squaredNorm();
        }

        return true;
    }

    bool InverseKinematicsNLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                           Ipopt::Number* grad_f)
    {
        if (new_x) {
#ifndef NDEBUG
            eval_f_called = false;
            eval_grad_f_called = false;
            eval_g_called = false;
            eval_jac_g_called = false;
#endif
            //First time we get called with this new value for the solution
            //Update the state and variables
            if (!updateState(x))
                return false;
        }
#ifndef NDEBUG
        assert(!eval_grad_f_called);
        eval_grad_f_called = true;
#endif

        Eigen::Map<Eigen::VectorXd> gradient(grad_f, n);
        gradient.setZero();

        //First part of the gradient: the part of the cost depending only on q_j
        Eigen::Map<Eigen::VectorXd> qj = iDynTree::toEigen(jointsAtOptimisationStep);
        Eigen::VectorXd jointError = iDynTree::toEigen(jointsAtOptimisationStep)-iDynTree::toEigen(m_data.m_preferredJointsConfiguration);

        Ipopt::Index baseSize = 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization);
        //last n - baseSize elements are the ones corresponding to qj
        iDynTree::VectorDynSize & jointCostWeight = m_data.m_preferredJointsWeight;
        for (size_t i = baseSize; i < n; ++i){
            gradient(i) = jointCostWeight(i - baseSize) * jointError(i - baseSize);
        }

        //Second part of the gradient: this part depends on all q, i.e. x
        //compute errors on rotation
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
                target != m_data.m_targets.end(); ++target) {
            if ((target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly ||
                    target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintNone)
                && target->second.hasPositionConstraint()) {
                //this implies that position is a soft constraint.
                iDynTree::Position positionError = targetsInfo[target->first].transform.getPosition() - target->second.getPosition();

                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                    computeConstraintJacobian(targetsInfo[target->first].jacobian,
                                              targetsInfo[target->first].quaternionDerivativeMap,
                                              quaternionDerivativeInverseMapBuffer,
                                              ComputeContraintJacobianOptionLinearPart,
                                              finalJacobianBuffer);
                } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
                    //RPY parametrization for the base
                    iDynTree::Vector3 rpy;
                    iDynTree::toEigen(rpy) = iDynTree::toEigen(this->optimizedBaseOrientation).head<3>();
                    iDynTree::Matrix3x3 RPYToOmega = iDynTree::Rotation::RPYRightTrivializedDerivative(rpy(0), rpy(1), rpy(2));

                    // RPY parametrization for the constraint
                    iDynTree::Vector3 rpy_target = targetsInfo[target->first].transform.getRotation().asRPY();
                    iDynTree::Matrix3x3 omegaToRPYMap_target = iDynTree::Rotation::RPYRightTrivializedDerivativeInverse(rpy_target(0), rpy_target(1), rpy_target(2));

                    computeConstraintJacobianRPY(targetsInfo[target->first].jacobian,
                                                 omegaToRPYMap_target,
                                                 RPYToOmega,
                                                 ComputeContraintJacobianOptionLinearPart,
                                                 finalJacobianBuffer);
                }


                //Note: transpose probably creates a temporary matrix. As position is
                //Fixedsize this should not fire a memory allocation in the heap, but we should
                //profile the code at some point
                gradient +=  target->second.getPositionWeight() * iDynTree::toEigen(positionError).transpose() * iDynTree::toEigen(finalJacobianBuffer).topRows<3>();

            }
            if ((target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly||
                    target->second.targetResolutionMode() == iDynTree::InverseKinematicsTreatTargetAsConstraintNone)
                && target->second.hasRotationConstraint()) {
                
                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                    //Derivative is (\tilde{Q} - 1) \partial_x Q
                    iDynTree::Rotation transformError = targetsInfo[target->first].transform.getRotation() * target->second.getRotation().inverse();

                    iDynTree::Vector4 orientationErrorQuaternion;
                    transformError.getQuaternion(orientationErrorQuaternion);

                    iDynTree::Vector4 identityQuaternion;
                    iDynTree::Rotation::Identity().getQuaternion(identityQuaternion);

                    //assert(m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion);


                    computeConstraintJacobian(targetsInfo[target->first].jacobian,
                                            targetsInfo[target->first].quaternionDerivativeMap,
                                            quaternionDerivativeInverseMapBuffer,
                                            ComputeContraintJacobianOptionAngularPart,
                                            finalJacobianBuffer);

                    //These are the first baseSize columns + the joint columns
                    gradient += target->second.getRotationWeight()*(iDynTree::toEigen(orientationErrorQuaternion) - iDynTree::toEigen(identityQuaternion)).transpose() * iDynTree::toEigen(finalJacobianBuffer).bottomRows(sizeOfRotationParametrization(m_data.m_rotationParametrization));
                }
                else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw){
                    //RPY parametrization for the base
                    iDynTree::Vector3 rpy;
                    iDynTree::toEigen(rpy) = iDynTree::toEigen(this->optimizedBaseOrientation).head<3>();
                    iDynTree::Matrix3x3 RPYToOmega = iDynTree::Rotation::RPYRightTrivializedDerivative(rpy(0), rpy(1), rpy(2));
                    FrameInfo &targetInfo = targetsInfo[target->first];

                    iDynTree::Rotation rotation_target = targetInfo.transform.getRotation();
                    iDynTree::Rotation rotation_desired = target->second.getRotation();
                    iDynTree::Rotation rotation_error;
                    iDynTree::toEigen(rotation_error) = iDynTree::toEigen(rotation_desired).transpose() * iDynTree::toEigen(rotation_target);
                    iDynTree::Vector3 rpy_error = rotation_error.asRPY();
                    iDynTree::Matrix3x3 omegaToRPYMap_target;
                    iDynTree::toEigen(omegaToRPYMap_target) = iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivativeInverse(rpy_error(0), rpy_error(1), rpy_error(2))) * iDynTree::toEigen(rotation_desired).transpose();

                    computeConstraintJacobianRPY(targetInfo.jacobian,
                                                omegaToRPYMap_target,
                                                RPYToOmega,
                                                ComputeContraintJacobianOptionAngularPart,
                                                finalJacobianBuffer);
                    //TODO Investigate the derivative of the cost using the RPY representation of the error matrix R*\hat{R}'
                    gradient += target->second.getRotationWeight()*( iDynTree::toEigen(rpy_error)).transpose() * iDynTree::toEigen(finalJacobianBuffer).bottomRows(sizeOfRotationParametrization(m_data.m_rotationParametrization));
                    
                }
            }
        }
        
        if (m_data.isCoMTargetActive() && !(m_data.isCoMaConstraint())) {
            assert(m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);
            iDynTree::Vector3 rpy;
            iDynTree::toEigen(rpy) = iDynTree::toEigen(this->optimizedBaseOrientation).head<3>();
            iDynTree::Matrix3x3 RPYToOmega = iDynTree::Rotation::RPYRightTrivializedDerivative(rpy(0), rpy(1), rpy(2));
            computeConstraintJacobianCOMRPY(comInfo.comJacobian, RPYToOmega, comInfo.comJacobianAnalytical);

            iDynTree::Position comPositionError;
            comPositionError = comInfo.com - m_data.m_comTarget.desiredPosition;
            gradient += m_data.m_comTarget.weight * iDynTree::toEigen(comPositionError).transpose() * iDynTree::toEigen(comInfo.comJacobianAnalytical);
        }

        return true;
    }

    bool InverseKinematicsNLP::eval_g(Ipopt::Index n, const Ipopt::Number* x,
                                      bool new_x, Ipopt::Index m, Ipopt::Number* g)
    {
        UNUSED_VARIABLE(n);
        if (new_x) {
#ifndef NDEBUG
            eval_f_called = false;
            eval_grad_f_called = false;
            eval_g_called = false;
            eval_jac_g_called = false;
#endif
            //First time we get called with this new value for the solution
            //Update the state and variables
            if (!updateState(x))
                return false;
        }
#ifndef NDEBUG
        assert(!eval_g_called);
        eval_g_called = true;
#endif

        Ipopt::Index index = 0;
        Eigen::Map<Eigen::VectorXd> constraints(g, m);
        //Start with the explicit constraints
        for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
            constraint != m_data.m_constraints.end(); ++constraint) {
            if (constraint->second.isActive()) {
                iDynTree::Transform &currentTransform = constraintsInfo[constraint->first].transform;

                if (constraint->second.hasPositionConstraint()) {
                    //get the position part
                    const iDynTree::Position &currentPosition = currentTransform.getPosition();
                    constraints.segment(index, 3) = iDynTree::toEigen(currentPosition);
                    index += 3;
                }
                if (constraint->second.hasRotationConstraint()) {
                    //get the rotation part
                    const iDynTree::Rotation &currentRotation = currentTransform.getRotation();
                    if (m_data.m_rotationParametrization ==
                        iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

                        //quaternion parametrization
                        iDynTree::Vector4 quaternionBuffer;
                        currentRotation.getQuaternion(quaternionBuffer);
                        constraints.segment(index, 4) = iDynTree::toEigen(quaternionBuffer);
                    } else if (m_data.m_rotationParametrization ==
                               iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {

                        currentRotation.getRPY(constraints(index), constraints(index + 1), constraints(index + 2));
                    }
                    index += sizeOfRotationParametrization(m_data.m_rotationParametrization);
                }
            }
        }

        // COM constraint
        if (m_data.m_comHullConstraint.isActive()) {
            constraints.segment(index, m_data.m_comHullConstraint.getNrOfConstraints()) =
                iDynTree::toEigen(m_data.m_comHullConstraint.A) * iDynTree::toEigen(comInfo.projectedCom);
            index += m_data.m_comHullConstraint.getNrOfConstraints();
        }
        
        if (m_data.isCoMTargetActive() && (m_data.isCoMaConstraint())){
            iDynTree::Position comPositionError;
            comPositionError = comInfo.com - m_data.m_comTarget.desiredPosition;
            constraints.segment<3>(index) = iDynTree::toEigen(comPositionError);
            index += 3;
        }

        //Targets considered as constraints
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
             target != m_data.m_targets.end(); ++target) {
            iDynTree::Transform &currentTransform = targetsInfo[target->first].transform;

            if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                && target->second.hasPositionConstraint()) {
                //add the position target as constraint
                const iDynTree::Position& currentPosition = currentTransform.getPosition();
                constraints.segment(index, 3) = iDynTree::toEigen(currentPosition);
                index += 3;
            }
            if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                && target->second.hasRotationConstraint()) {
                //Add the orientation target as constraint
                const iDynTree::Rotation& currentRotation = currentTransform.getRotation();
                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

                    //quaternion parametrization
                    iDynTree::Vector4 quaternionBuffer;
                    currentRotation.getQuaternion(quaternionBuffer);
                    constraints.segment(index, 4) = iDynTree::toEigen(quaternionBuffer);

                } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {

                    //RPY parametrization
                    currentRotation.getRPY(constraints(index), constraints(index + 1), constraints(index + 2));
                }
                index += sizeOfRotationParametrization(m_data.m_rotationParametrization);            }
        }

        //Constraint on the orientation of the base if using quaternion
        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            //Quaternion norm constraint
            // || Q(base) ||^2 = 1
            constraints[index++] = iDynTree::toEigen(this->optimizedBaseOrientation).squaredNorm();
        }

        assert(index == m);
        return true;
    }

    bool InverseKinematicsNLP::eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                                          Ipopt::Index *jCol, Ipopt::Number* values)
    {
        if (!values) {
            //Define the sparsity pattern of the jacobian
            Ipopt::Index index = 0;

            for (Ipopt::Index row = 0; row < m; row++) {
                auto columnIndices = m_jacobianSparsityHelper.nonZeroIndicesForRow(row);
                size_t numberOfPreviousNonzeros = m_jacobianSparsityHelper.totalNumberOfNonZerosBeforeRow(row);
                for (size_t col = 0; col < columnIndices.size(); ++col) {
                    iRow[numberOfPreviousNonzeros + col] = row;
                    jCol[numberOfPreviousNonzeros + col] = columnIndices[col];
                    ++index;
                }
            }
            assert(nele_jac == index);

        } else {
            
            //This is called every time
            if (new_x) {
#ifndef NDEBUG
                eval_f_called = false;
                eval_grad_f_called = false;
                eval_g_called = false;
                eval_jac_g_called = false;
#endif
                //First time we get called with this new value for the solution
                //Update the state and variables
                if (!updateState(x))
                    return false;
            }
#ifndef NDEBUG
            assert(!eval_jac_g_called);
            eval_jac_g_called = true;
#endif
            Ipopt::Index constraintIndex = 0;

            for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
                 constraint != m_data.m_constraints.end(); ++constraint) {

                if (constraint->second.isActive()) {
                    //For each constraint compute its jacobian
                    FrameInfo &constraintInfo = constraintsInfo[constraint->first];

                    if (m_data.m_rotationParametrization ==
                        iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

                        //We adapt the jacobian to handle two things
                        // - the orientation part is represented as a quaternion
                        // - the iDynTree Jacobian gives the corresponding velocity in term of
                        //   angular velocity. Here the derivative is not w.r.t. time
                        //   but w.r.t. an arbitrary variation of the configuration
                        computeConstraintJacobian(constraintInfo.jacobian,
                                                  constraintInfo.quaternionDerivativeMap,
                                                  quaternionDerivativeInverseMapBuffer,
                                                  ComputeContraintJacobianOptionLinearPart |
                                                  ComputeContraintJacobianOptionAngularPart,
                                                  finalJacobianBuffer);

                    } else if (m_data.m_rotationParametrization ==
                               iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
                        //RPY parametrization for the base
                        iDynTree::Vector3 rpy;
                        iDynTree::toEigen(rpy) = iDynTree::toEigen(this->optimizedBaseOrientation).head<3>();
                        iDynTree::Matrix3x3 RPYToOmega = iDynTree::Rotation::RPYRightTrivializedDerivative(rpy(0),
                                                                                                           rpy(1),
                                                                                                           rpy(2));

                        // RPY parametrization for the constraint
                        iDynTree::Vector3 rpy_target = constraintInfo.transform.getRotation().asRPY();
                        iDynTree::Matrix3x3 omegaToRPYMap_target = iDynTree::Rotation::RPYRightTrivializedDerivativeInverse(
                                rpy_target(0), rpy_target(1), rpy_target(2));

                        computeConstraintJacobianRPY(constraintInfo.jacobian,
                                                     omegaToRPYMap_target,
                                                     RPYToOmega,
                                                     ComputeContraintJacobianOptionLinearPart |
                                                     ComputeContraintJacobianOptionAngularPart,
                                                     finalJacobianBuffer);
                    }


                    //Now that we computed the actual Jacobian needed by IPOPT
                    //We have to assign it to the correct variable
                    if (constraint->second.hasPositionConstraint()) {
                        //Position part
                        m_jacobianSparsityHelper.assignActualMatrixValues({constraintIndex, 3},
                                                                          finalJacobianBuffer, 0,
                                                                          values);
                        constraintIndex += 3;
                    }
                    if (constraint->second.hasRotationConstraint()) {
                        //Orientation part
                        m_jacobianSparsityHelper.assignActualMatrixValues({constraintIndex,
                                                                           sizeOfRotationParametrization(m_data.m_rotationParametrization)},
                                                                          finalJacobianBuffer, 3,
                                                                          values);
                        constraintIndex += sizeOfRotationParametrization(m_data.m_rotationParametrization);
                    }
                }
            }

            //For COM constraint
            //RPY parametrization for the base
            if (m_data.m_comHullConstraint.isActive() || (m_data.isCoMTargetActive() && m_data.isCoMaConstraint())) {
                assert(m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);
                iDynTree::Vector3 rpy;
                iDynTree::toEigen(rpy) = iDynTree::toEigen(this->optimizedBaseOrientation).head<3>();
                iDynTree::Matrix3x3 RPYToOmega = iDynTree::Rotation::RPYRightTrivializedDerivative(rpy(0), rpy(1), rpy(2));
                computeConstraintJacobianCOMRPY(comInfo.comJacobian, RPYToOmega, comInfo.comJacobianAnalytical);
                
                if (m_data.m_comHullConstraint.isActive()) {
                    // Project the jacobian
                    iDynTree::toEigen(comInfo.projectedComJacobian) =
                            iDynTree::toEigen(m_data.m_comHullConstraint.A) * iDynTree::toEigen(m_data.m_comHullConstraint.Pdirection) * iDynTree::toEigen(comInfo.comJacobianAnalytical);

                    m_jacobianSparsityHelper.assignActualMatrixValues({constraintIndex, static_cast<ptrdiff_t>(comInfo.projectedComJacobian.rows())},
                                                                      comInfo.projectedComJacobian, 0,
                                                                      values);

                    constraintIndex += comInfo.projectedComJacobian.rows();
                }
                
                if (m_data.isCoMTargetActive()) {
                    m_jacobianSparsityHelper.assignActualMatrixValues({constraintIndex, static_cast<ptrdiff_t>(comInfo.comJacobianAnalytical.rows())},
                                                                      comInfo.comJacobianAnalytical, 0,
                                                                      values);
                    constraintIndex += 3;
                }
            }


            //For all targets enforced as constraints
            for (TransformMap::const_iterator target = m_data.m_targets.begin();
                 target != m_data.m_targets.end(); ++target) {

                FrameInfo &targetInfo = targetsInfo[target->first];

                //Depending if we need position and/or orientation
                //we have to adapt different parts of the jacobian
                int computationOption = 0;
                if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly)
                    computationOption |= ComputeContraintJacobianOptionLinearPart;
                if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly)
                    computationOption |= ComputeContraintJacobianOptionAngularPart;

                if (computationOption == 0) continue; // no need for further computations

                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

                    //We adapt the jacobian to handle two things
                    // - the orientation part is represented as a quaternion
                    // - the iDynTree Jacobian gives the corresponding velocity in term of
                    //   angular velocity. Here the derivative is not w.r.t. time
                    //   but w.r.t. an arbitrary variation of the configuration
                    computeConstraintJacobian(targetInfo.jacobian,
                                              targetInfo.quaternionDerivativeMap,
                                              quaternionDerivativeInverseMapBuffer,
                                              computationOption,
                                              finalJacobianBuffer);

                } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
                    //RPY parametrization for the base
                    iDynTree::Vector3 rpy;
                    iDynTree::toEigen(rpy) = iDynTree::toEigen(this->optimizedBaseOrientation).head<3>();
                    iDynTree::Matrix3x3 RPYToOmega = iDynTree::Rotation::RPYRightTrivializedDerivative(rpy(0), rpy(1), rpy(2));

                    // RPY parametrization for the constraint
                    iDynTree::Vector3 rpy_target = targetInfo.transform.getRotation().asRPY();
                    iDynTree::Matrix3x3 omegaToRPYMap_target = iDynTree::Rotation::RPYRightTrivializedDerivativeInverse(rpy_target(0), rpy_target(1), rpy_target(2));

                    computeConstraintJacobianRPY(targetInfo.jacobian,
                                                 omegaToRPYMap_target,
                                                 RPYToOmega,
                                                 computationOption,
                                                 finalJacobianBuffer);
                }

                if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                    && target->second.hasPositionConstraint()) {

                    //Copy position part
                    m_jacobianSparsityHelper.assignActualMatrixValues({constraintIndex, 3},
                                                                      finalJacobianBuffer, 0,
                                                                      values);
                    constraintIndex += 3;
                }

                if (target->second.targetResolutionMode() & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                    && target->second.hasRotationConstraint()) {
                    //Orientation part
                    m_jacobianSparsityHelper.assignActualMatrixValues({constraintIndex,
                        sizeOfRotationParametrization(m_data.m_rotationParametrization)},
                                                                      finalJacobianBuffer, 3,
                                                                      values);
                    constraintIndex += sizeOfRotationParametrization(m_data.m_rotationParametrization);
                }
            }


            //Finally, the norm of the base orientation quaternion parametrization
            if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                //Quaternion norm derivative
                // = 2 * Q^\top
                Eigen::Map<Eigen::VectorXd> quaternionDerivative(&values[constraintIndex * n], 4);
                quaternionDerivative = 2 * iDynTree::toEigen(this->optimizedBaseOrientation).transpose();
                constraintIndex++;
            }

            assert(m == constraintIndex);

        }
        return true;
    }

    bool InverseKinematicsNLP::eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                      Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                                      Ipopt::Index* jCol, Ipopt::Number* values)
    {
        if (new_x) {
            if (!updateState(x))
                return false;
        }
        //TODO: hessian of Lagrangian
        return false;
    }

    void InverseKinematicsNLP::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                                 const Ipopt::Number* x, const Ipopt::Number* z_L,
                                                 const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                                 const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                                 const Ipopt::IpoptData* ip_data,
                                                 Ipopt::IpoptCalculatedQuantities* ip_cq)
    {
        //TODO: save the status


        //Obtain base position
        iDynTree::Position basePosition;
        basePosition(0) = x[0];
        basePosition(1) = x[1];
        basePosition(2) = x[2];

        //Obtain base orientation
        iDynTree::Vector4 baseOrientationSerialization;
        for (unsigned i = 0; i < sizeOfRotationParametrization(m_data.m_rotationParametrization); ++i) {
            baseOrientationSerialization(i) = x[3 + i];
        }

        iDynTree::Rotation baseOrientation = iDynTree::Rotation::Identity();
        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            baseOrientation.fromQuaternion(baseOrientationSerialization);

        } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
            baseOrientation = iDynTree::Rotation::RPY(baseOrientationSerialization(0),
                                                      baseOrientationSerialization(1),
                                                      baseOrientationSerialization(2));
        }
        //Save base pose
        m_data.m_baseResults.setPosition(basePosition);
        m_data.m_baseResults.setRotation(baseOrientation);

        //joints
        Eigen::Map<const Eigen::VectorXd> optimisedJoints(&x[3 + sizeOfRotationParametrization(m_data.m_rotationParametrization)], m_data.m_dofs);
        iDynTree::toEigen(m_data.m_jointsResults) = optimisedJoints;

        // additional info: multipliers
        Eigen::Map<const Eigen::VectorXd> multipliers(lambda, m);
        iDynTree::toEigen(m_data.m_constraintMultipliers) = multipliers;

        Eigen::Map<const Eigen::VectorXd> lowerBounds(z_L, n);
        Eigen::Map<const Eigen::VectorXd> upperBounds(z_U, n);
        iDynTree::toEigen(m_data.m_lowerBoundMultipliers) = lowerBounds;
        iDynTree::toEigen(m_data.m_upperBoundMultipliers) = upperBounds;

    }

    Ipopt::Index InverseKinematicsNLP::get_number_of_nonlinear_variables()
    {
        /*
         This method is only important if the limited-memory quasi-Newton options is used, see Section 4.2. It is used to return the number of variables that appear nonlinearly in the objective function or in at least one constraint function. If a negative number is returned, IPOPT assumes that all variables are nonlinear.

         If the user doesn't overload this method in her implementation of the class derived from TNLP, the default implementation returns -1, i.e., all variables are assumed to be nonlinear.
         */
        //this is tied to the Jacobian sparsity
        return -1;
    }

    bool InverseKinematicsNLP::get_list_of_nonlinear_variables(Ipopt::Index num_nonlin_vars,
                                                               Ipopt::Index* pos_nonlin_vars)
    {
        /*
         This method is called by IPOPT only if the limited-memory quasi-Newton options is used and if the get_number_of_nonlinear_variables method returns a positive number; this number is then identical with num_nonlin_vars and the length of the array pos_nonlin_vars. In this call, you need to list the indices of all nonlinear variables in pos_nonlin_vars, where the numbering starts with 0 order 1, depending on the numbering style determined in get_nlp_info.
         */
        return true;
    }



    void InverseKinematicsNLP::computeConstraintJacobian(const iDynTree::MatrixDynSize& transformJacobianBuffer,
                                                         const iDynTree::MatrixFixSize<4, 3>& _quaternionDerivativeMap,
                                                         const iDynTree::MatrixFixSize<3, 4>& _quaternionDerivativeInverseMap,
                                                         const int computationOption,
                                                         iDynTree::MatrixDynSize& constraintJacobianBuffer)
    {
        Eigen::Map<const Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > quaternionDerivativeMap = iDynTree::toEigen(_quaternionDerivativeMap);
        Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > quaternionDerivativeInverseMap = iDynTree::toEigen(_quaternionDerivativeInverseMap);

        //Number of joints
        unsigned n = constraintJacobianBuffer.cols() - 7;

        constraintJacobianBuffer.zero();

        //I have to obtain a Jacobian in 7 x 7 + dofs
        iDynTree::iDynTreeEigenConstMatrixMap frameJacobian = iDynTree::toEigen(transformJacobianBuffer);
        iDynTree::iDynTreeEigenMatrixMap constraintJacobian = iDynTree::toEigen(constraintJacobianBuffer);

        //This implement Eq. 16 of the IK document
        if (computationOption & ComputeContraintJacobianOptionLinearPart) {
            //Position (linear) part of the Jacobian
            constraintJacobian.topLeftCorner<3, 3>() = frameJacobian.topLeftCorner<3, 3>();
            constraintJacobian.block<3, 4>(0, 3) = frameJacobian.block<3, 3>(0, 3) * quaternionDerivativeInverseMap;
            constraintJacobian.topRightCorner(3, n) = frameJacobian.topRightCorner(3, n);
        }

        if (computationOption & ComputeContraintJacobianOptionAngularPart) {
            //Angular part of the Jacobian
            constraintJacobian.bottomLeftCorner<4, 3>() = quaternionDerivativeMap * frameJacobian.bottomLeftCorner<3, 3>();
            constraintJacobian.block<4, 4>(3, 3) = quaternionDerivativeMap * frameJacobian.block<3, 3>(3, 3) * quaternionDerivativeInverseMap;
            constraintJacobian.bottomRightCorner(4, n) = quaternionDerivativeMap * frameJacobian.bottomRightCorner(3, n);
        }
    }

    void InverseKinematicsNLP::computeConstraintJacobianRPY(const iDynTree::MatrixDynSize& transformJacobianBuffer,
                                                            const iDynTree::MatrixFixSize<3, 3>& _rpyDerivativeMap,
                                                            const iDynTree::MatrixFixSize<3, 3>& _rpyDerivativeInverseMap,
                                                            const int computationOption,
                                                            iDynTree::MatrixDynSize& constraintJacobianBuffer)
    {
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rpyDerivativeMap = iDynTree::toEigen(_rpyDerivativeMap);
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rpyDerivativeInverseMap = iDynTree::toEigen(_rpyDerivativeInverseMap);

        //Number of joints
        unsigned n = constraintJacobianBuffer.cols() - 6;

        constraintJacobianBuffer.zero();

        iDynTree::iDynTreeEigenConstMatrixMap frameJacobian = iDynTree::toEigen(transformJacobianBuffer);
        iDynTree::iDynTreeEigenMatrixMap constraintJacobian = iDynTree::toEigen(constraintJacobianBuffer);

        if (computationOption & ComputeContraintJacobianOptionLinearPart) {
            //Position (linear) part of the Jacobian
            constraintJacobian.topLeftCorner<3, 3>() = frameJacobian.topLeftCorner<3, 3>();
            constraintJacobian.block<3, 3>(0, 3) = frameJacobian.block<3, 3>(0, 3) * rpyDerivativeInverseMap;
            constraintJacobian.topRightCorner(3, n) = frameJacobian.topRightCorner(3, n);
        }

        if (computationOption & ComputeContraintJacobianOptionAngularPart) {
            //Angular part of the Jacobian
            constraintJacobian.bottomLeftCorner<3, 3>() = rpyDerivativeMap * frameJacobian.bottomLeftCorner<3, 3>();
            constraintJacobian.block<3, 3>(3, 3) = rpyDerivativeMap * frameJacobian.block<3, 3>(3, 3) * rpyDerivativeInverseMap;
            constraintJacobian.bottomRightCorner(3, n) = rpyDerivativeMap * frameJacobian.bottomRightCorner(3, n);
        }
    }

    void InverseKinematicsNLP::computeConstraintJacobianCOMRPY(const iDynTree::MatrixDynSize& comJacobianBuffer,
                                                               const iDynTree::MatrixFixSize<3, 3>& _rpyDerivativeInverseMap,
                                                                     iDynTree::MatrixDynSize& constraintComJacobianBuffer)
    {
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rpyDerivativeInverseMap = iDynTree::toEigen(_rpyDerivativeInverseMap);

        //Number of joints
        unsigned n = constraintComJacobianBuffer.cols() - 6;

        constraintComJacobianBuffer.zero();

        iDynTree::iDynTreeEigenConstMatrixMap comJacobian = iDynTree::toEigen(comJacobianBuffer);
        iDynTree::iDynTreeEigenMatrixMap constraintJacobian = iDynTree::toEigen(constraintComJacobianBuffer);


        //Position (linear) part of the Jacobian
        constraintJacobian.topLeftCorner<3, 3>() = comJacobian.topLeftCorner<3, 3>();
        constraintJacobian.block<3, 3>(0, 3) = comJacobian.block<3, 3>(0, 3) * rpyDerivativeInverseMap;
        constraintJacobian.topRightCorner(3, n) = comJacobian.topRightCorner(3, n);
    }

    void InverseKinematicsNLP::omegaToRPYParameters(const iDynTree::Vector3& rpyAngles,
                                                    iDynTree::Matrix3x3 &map)
    {
        /*
         {}^I omega =
         [ cos(psi)*cos(theta), -sin(psi), 0]
         [ cos(theta)*sin(psi),  cos(psi), 0]
         [          sin(theta),         0, 1]

        d/dt(rpy)

         We thus look for the inverse of this matrix
         */

        map.zero();

        double detMapMatrix = std::cos(rpyAngles(1));

        /*
         inverse is
         [                                                                              cos(psi),                                                                             sin(psi),                                             0]
         [ -(sin(psi)*(cos(psi)^2*cos(theta) + cos(theta)*sin(psi)^2))/(cos(psi)^2 + sin(psi)^2), (cos(psi)*(cos(psi)^2*cos(theta) + cos(theta)*sin(psi)^2))/(cos(psi)^2 + sin(psi)^2),                                             0]
         [                                                                  -cos(psi)*sin(theta),                                                                 -sin(psi)*sin(theta), cos(psi)^2*cos(theta) + cos(theta)*sin(psi)^2]
         */

        map(0, 0) = std::cos(rpyAngles(2)) / detMapMatrix;
        map(0, 1) = std::sin(rpyAngles(2)) / detMapMatrix;
        map(1, 0) = -std::sin(rpyAngles(2));
        map(1, 1) = std::cos(rpyAngles(2));
        map(2, 0) = -std::cos(rpyAngles(2)) * std::tan(rpyAngles(1));
        map(2, 1) = -std::sin(rpyAngles(2)) * std::tan(rpyAngles(1));
        map(2, 2) = 1;

    }


    bool InverseKinematicsNLP::intermediate_callback(Ipopt::AlgorithmMode mode,
                                                     Ipopt::Index iter, Ipopt::Number obj_value,
                                                     Ipopt::Number inf_pr, Ipopt::Number inf_du,
                                                     Ipopt::Number mu, Ipopt::Number d_norm,
                                                     Ipopt::Number regularization_size,
                                                     Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                                                     Ipopt::Index ls_trials,
                                                     const Ipopt::IpoptData* ip_data,
                                                     Ipopt::IpoptCalculatedQuantities* ip_cq)
    {
//        std::cerr << "Iteration " << iter << "\n";
//        for (TransformMap::const_iterator target = m_data.m_targets.begin();
//             target != m_data.m_targets.end(); ++target) {
//            std::cerr << "Target " << target->first << "\n";
//            std::cerr << targetsInfo[target->first].transform.toString() << "\n";
//        }
//
//        for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
//             constraint != m_data.m_constraints.end(); ++constraint) {
//            std::cerr << "Constraint " << constraint->first << "\n";
//            std::cerr << constraintsInfo[constraint->first].transform.toString() << "\n";
//        }
//


        return true;
    }

    void InverseKinematicsNLP::testDerivatives(const iDynTree::VectorDynSize& derivativePoint, int frameIndex, double epsilon, double tolerance, int _parametrization)
    {
        //Note: not updated afer migration

        using namespace Ipopt;
        using namespace iDynTree;
        using namespace Eigen;

        enum InverseKinematicsRotationParametrization parametrization = (enum InverseKinematicsRotationParametrization)_parametrization;

        //Copying initial point
        Number *_x = new Number[derivativePoint.size()];
        Map<VectorXd> x(_x, derivativePoint.size());
        x = toEigen(derivativePoint);

        if (parametrization == InverseKinematicsRotationParametrizationQuaternion) {
            std::cerr << "Quaternion\n" << x.segment<4>(3).transpose() << "\n";
        } else if (parametrization == InverseKinematicsRotationParametrizationRollPitchYaw) {
//            std::cerr << "RPY\n" << (x.segment<3>(3).transpose() * 180.0 / M_PI) << "\n";
        }


        //Compute analytical derivatives
        MatrixDynSize _analyticalJacobian(3 + sizeOfRotationParametrization(parametrization), derivativePoint.size());
        _analyticalJacobian.zero();
        Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > analyticalJacobian = toEigen(_analyticalJacobian);

        updateState(_x);

        MatrixDynSize dynTreeJacobian(6, 6 + derivativePoint.size() - (3 + sizeOfRotationParametrization(parametrization)));
        m_data.m_dynamics.getFrameFreeFloatingJacobian(frameIndex, dynTreeJacobian);

        std::cerr << "Jacobian (iDynTree)\n" << dynTreeJacobian.toString() << "\n";

        if (parametrization == InverseKinematicsRotationParametrizationQuaternion) {
            iDynTree::MatrixFixSize<4, 3> quaternionDerivativeMapBuffer;
            computeConstraintJacobian(dynTreeJacobian,
                                      quaternionDerivativeMapBuffer,
                                      quaternionDerivativeInverseMapBuffer,
                                      ComputeContraintJacobianOptionLinearPart|ComputeContraintJacobianOptionAngularPart,
                                      _analyticalJacobian);
        } else if (parametrization == InverseKinematicsRotationParametrizationRollPitchYaw) {
            analyticalJacobian = toEigen(dynTreeJacobian);
            iDynTree::Transform currentTransform = m_data.m_dynamics.getWorldTransform(frameIndex);
            Vector3 rpy;
            currentTransform.getRotation().getRPY(rpy(0), rpy(1), rpy(2));
            std::cerr << "RPY\n" << rpy.toString() << "\n";

            Matrix3x3 map;
            map.zero();
            omegaToRPYParameters(rpy, map);
            analyticalJacobian.bottomRows<3>() = toEigen(map) * analyticalJacobian.bottomRows<3>();
            std::cerr << "Map\n" << map.toString() << "\n";
        }


        //Compute numerical derivative
        MatrixDynSize _finiteDifferenceJacobian(3 + sizeOfRotationParametrization(parametrization), derivativePoint.size());
        _finiteDifferenceJacobian.zero();
        Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > finiteDifferenceJacobian = toEigen(_finiteDifferenceJacobian);

        Number * _xPlusIncrement = new Number[derivativePoint.size()];
        Map<VectorXd> xPlusIncrement(_xPlusIncrement, derivativePoint.size());

        for (unsigned i = 0; i < derivativePoint.size(); ++i) {
            // x + Dx
            xPlusIncrement = x;
            xPlusIncrement(i) += epsilon;

            VectorXd positiveIncrement(3 + sizeOfRotationParametrization(parametrization));
            positiveIncrement.setZero();
            {
                updateState(_xPlusIncrement);
                iDynTree::Transform currentTransform = m_data.m_dynamics.getWorldTransform(frameIndex);

                const iDynTree::Position& currentPosition = currentTransform.getPosition();
                positiveIncrement.head<3>() = iDynTree::toEigen(currentPosition);

                const iDynTree::Rotation& currentRotation = currentTransform.getRotation();
                if (parametrization == InverseKinematicsRotationParametrizationQuaternion) {
                    //get quaternion
                    Vector4 quaternion;
                    currentRotation.getQuaternion(quaternion);
                    positiveIncrement.tail<4>() = iDynTree::toEigen(quaternion);
                } else if (parametrization == InverseKinematicsRotationParametrizationRollPitchYaw) {
                    //get quaternion
                    Vector3 rpy;
                    currentRotation.getRPY(rpy(0), rpy(1), rpy(2));
                    positiveIncrement.tail<3>() = iDynTree::toEigen(rpy);
                }

            }
            // x - Dx
            xPlusIncrement = x;
            xPlusIncrement(i) -= epsilon;

            VectorXd negativeIncrement(3 + sizeOfRotationParametrization(parametrization));
            negativeIncrement.setZero();

            {
                updateState(_xPlusIncrement);
                iDynTree::Transform currentTransform = m_data.m_dynamics.getWorldTransform(frameIndex);

                const iDynTree::Position& currentPosition = currentTransform.getPosition();
                negativeIncrement.head<3>() = iDynTree::toEigen(currentPosition);

                const iDynTree::Rotation& currentRotation = currentTransform.getRotation();
                if (parametrization == InverseKinematicsRotationParametrizationQuaternion) {
                    //get quaternion
                    Vector4 quaternion;
                    currentRotation.getQuaternion(quaternion);
                    negativeIncrement.tail<4>() = iDynTree::toEigen(quaternion);
                    //                std::cerr << "Quat-:\t" << quaternion.toString() << "\n";
                } else if (parametrization == InverseKinematicsRotationParametrizationRollPitchYaw) {
                    //get quaternion
                    Vector3 rpy;
                    currentRotation.getRPY(rpy(0), rpy(1), rpy(2));
                    negativeIncrement.tail<3>() = iDynTree::toEigen(rpy);
                }
            }

            finiteDifferenceJacobian.col(i) = (positiveIncrement - negativeIncrement) / (2 * epsilon);

        }

        std::cerr << "**************************\n";
        std::cerr << "Derivative Test\n";
        std::cerr << "**************************\n\n";

        std::cerr << "Analytical Jacobian (Base)\n";
        std::cerr << analyticalJacobian.leftCols(3 + sizeOfRotationParametrization(parametrization)) << "\n\n";
        std::cerr << "Finite Difference Jacobian (Base)\n";
        std::cerr << finiteDifferenceJacobian.leftCols(3 + sizeOfRotationParametrization(parametrization)) << "\n\n\n";
        std::cerr << "Analytical Jacobian (Joints)\n";
        std::cerr << analyticalJacobian.rightCols(derivativePoint.size() - (3 + sizeOfRotationParametrization(parametrization))) << "\n\n";
        std::cerr << "Finite Difference Jacobian (Joints)\n";
        std::cerr << finiteDifferenceJacobian.rightCols(derivativePoint.size() - (3 + sizeOfRotationParametrization(parametrization))) << "\n\n\n";

        std::cerr << std::scientific;
        std::cerr.precision(7);

        //Checking error
        for (size_t i = 0; i < derivativePoint.size(); ++i) {
            VectorXd error = analyticalJacobian.col(i) - finiteDifferenceJacobian.col(i);
            for (unsigned row = 0; row < (3 + sizeOfRotationParametrization(parametrization)); ++row) {
                if (std::abs(error(row)) > tolerance) {
                    std::cerr << "[" << row << "," << i << "] ";
                    std::cerr << analyticalJacobian(row, i) << "\t" << finiteDifferenceJacobian(row, i);
                    std::cerr << "\t[" << error(row) << "]\n";
                }
            }
        }

        std::cerr.unsetf(std::ios_base::floatfield);

        delete [] _xPlusIncrement;
        delete [] _x;


    }
}
}
