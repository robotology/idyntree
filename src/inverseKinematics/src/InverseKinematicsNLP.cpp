/*!
 * @file InverseKinematicsNLP.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */

#include "InverseKinematicsNLP.h"

#include "InverseKinematicsData.h"
#include "Transform.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <cassert>
#include <cmath>

namespace internal {
namespace kinematics {

    //To understand IPOpt log:
    //http://www.coin-or.org/Ipopt/documentation/node36.html#sec:output

    InverseKinematicsNLP::InverseKinematicsNLP(InverseKinematicsData& data)
    : m_data(data)
    , jointCostWeight(1e-2)
    { }

    InverseKinematicsNLP::~InverseKinematicsNLP() {}


    void InverseKinematicsNLP::initializeInternalData(Ipopt::Index n, Ipopt::Index m)
    {
        UNUSED_VARIABLE(m);
        //This method should initialize all the internal buffers used during
        //the optimization

        jointsConfiguration = m_data.m_state.jointsConfiguration; //copy the vector
        optimizedJoints.resize(n - (3 + sizeOfRotationParametrization(m_data.m_rotationParametrization)));

        //resize some buffers
        finalJacobianBuffer.resize((3 + sizeOfRotationParametrization(m_data.m_rotationParametrization)), n);

        constraintsInfo.clear();
        targetsInfo.clear();

        //prepare buffers for constraints and targets
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
             target != m_data.m_targets.end(); ++target) {
            FrameInfo info;
            info.jacobian.resize(6, m_data.m_dofs + 6);
            targetsInfo.insert(FrameInfoMap::value_type(target->first, info));
        }

        for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
             constraint != m_data.m_constraints.end(); ++constraint) {
            FrameInfo info;
            info.jacobian.resize(6, m_data.m_dofs + 6);
            constraintsInfo.insert(FrameInfoMap::value_type(constraint->first, info));
        }
    }

    bool InverseKinematicsNLP::updateState(const Ipopt::Number * x)
    {
        //This method computes all the data which is needed in more than one place.

        //first: save robot configuration
        //position
        basePosition(0) = x[0];
        basePosition(1) = x[1];
        basePosition(2) = x[2];

        //orientation
        for (unsigned i = 0; i < sizeOfRotationParametrization(m_data.m_rotationParametrization); ++i) {
            this->baseOrientation(i) = x[3 + i];
        }

        //joints
        for (unsigned index = 0; index < m_data.m_dofs; ++index) {
            this->optimizedJoints(index) = x[3 + sizeOfRotationParametrization(m_data.m_rotationParametrization) + index];
            this->optimizedJoints(index) = jointsConfiguration(index);
        }

        //Update robot state and state-dependent variables
        iDynTree::Rotation baseOrientationRotation;

        //Get the rotation from the used serialization
        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            //Quaternion parametrization
            baseOrientationRotation.fromQuaternion(this->baseOrientation);

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
            iDynTree::Vector4 normQuaternioniDyn = this->baseOrientation;
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
            Eigen::Map<Eigen::Vector4d> quaternion = iDynTree::toEigen(this->baseOrientation);
            double quaternionSNorm = quaternion.squaredNorm();
            normalizedQuaternionDerivative.setIdentity();
            normalizedQuaternionDerivative *= quaternionSNorm;
            normalizedQuaternionDerivative -= (quaternion * quaternion.transpose());
            normalizedQuaternionDerivative /= std::pow(quaternionSNorm, 3.0/2.0);

            inverseMap *= normalizedQuaternionDerivative;

        } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
            //RPY parametrization
            baseOrientationRotation = iDynTree::Rotation::RPY(this->baseOrientation(0), this->baseOrientation(1), this->baseOrientation(2));
        }
        // base pose
        iDynTree::Transform basePose(baseOrientationRotation, basePosition);

        if (!m_data.m_dynamics.setRobotState(basePose,
                                             jointsConfiguration,
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

            FrameInfo &frameInfo = constraintsInfo[constraint->first];
            frameInfo.transform = m_data.m_dynamics.getWorldTransform(constraint->first);
            m_data.m_dynamics.getFrameFreeFloatingJacobian(constraint->first, frameInfo.jacobian);

            iDynTree::Vector4 transformQuat;
            frameInfo.transform.getRotation().getQuaternion(transformQuat);
            //compute quaternionDerivativeMapBuffer
            //See in target for details on the computation
            Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > map = iDynTree::toEigen(frameInfo.quaternionDerivativeMap);
            map.topRows<1>() = -iDynTree::toEigen(transformQuat).tail<3>().transpose();
            map.bottomRows<3>().setIdentity();
            map.bottomRows<3>() *= transformQuat(0);
            map.bottomRows<3>() -= iDynTree::skew(iDynTree::toEigen(transformQuat).tail<3>());

            map *= 0.5;
        }

        return true;
    }

    bool InverseKinematicsNLP::get_nlp_info(Ipopt::Index& n,
                                            Ipopt::Index& m,
                                            Ipopt::Index& nnz_jac_g,
                                            Ipopt::Index& nnz_h_lag,
                                            IndexStyleEnum& index_style)
    {
        //Size of optimization variables is 3 + Orientation (base) + size of joints we optimize
        n = 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization) + m_data.m_optimizedRobotDofs.size();

        //Start adding constraints
        m = 0;
        for (TransformMap::const_iterator it = m_data.m_constraints.begin();
             it != m_data.m_constraints.end(); ++it) {
            //Frame constraint: it can have position, rotation or both elements
            if (it->second.hasPositionConstraint()) {
                m += 3;
            }
            if (it->second.hasRotationConstraint()) {
                m += sizeOfRotationParametrization(m_data.m_rotationParametrization);
            }
        }

        //add target if considered as constraints
        for (TransformMap::const_iterator it = m_data.m_targets.begin();
             it != m_data.m_targets.end(); ++it) {
            if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                && it->second.hasPositionConstraint()) {
                m += 3;
            }
            if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                && it->second.hasRotationConstraint()) {

                m += sizeOfRotationParametrization(m_data.m_rotationParametrization);;
            }
        }

        //TODO: implement in iDynTree the sparsity pattern of Jacobian and Hessians
        nnz_jac_g = m * n;

        if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
            //If the rotation is parametrized as quaternion
            //that the base orientation yields an additional
            //constraint, i.e. norm of quaternion = 1
            m += 1; //quaternion norm constraint
            nnz_jac_g += 4; //quaternion norm is sparse.
        }

        nnz_h_lag = n * n; //this is currently ignored

        index_style = C_STYLE;

        initializeInternalData(n, m);

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
            const std::pair<double, double> &limits = m_data.m_jointLimits[i - baseSize];
            x_l[i] = limits.first;
            x_u[i] = limits.second;
        }

        //Equality constraints
        Ipopt::Index constraintIndex = 0;
        for (TransformMap::const_iterator it = m_data.m_constraints.begin();
             it != m_data.m_constraints.end(); it++) {

            //This is a constraint on a frame position
            if (it->second.hasPositionConstraint()) {
                const iDynTree::Position& position = it->second.getPosition();
                g_l[constraintIndex] = g_u[constraintIndex] = position(0);
                constraintIndex++;
                g_l[constraintIndex] = g_u[constraintIndex] = position(1);
                constraintIndex++;
                g_l[constraintIndex] = g_u[constraintIndex] = position(2);
                constraintIndex++;
            }
            //This is a constraint on a frame orientation
            if (it->second.hasRotationConstraint()) {
                const iDynTree::Rotation& rotation = it->second.getRotation();

                //Get the desired orientation in its representation
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

        //target <=> position constraint
        for (TransformMap::const_iterator it = m_data.m_targets.begin();
             it != m_data.m_targets.end(); ++it) {
            if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
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
            if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
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
            for (Ipopt::Index i = 0; i < 3; ++i) {
                x[i] = m_data.m_optimizedBasePosition(i);
            }
            for (Ipopt::Index i = 0; i < sizeOfRotationParametrization(m_data.m_rotationParametrization); ++i) {
                x[3 + i] = m_data.m_optimizedBaseOrientation(i);
            }

            Ipopt::Index baseSize = 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization);
            for (Ipopt::Index i = 0; i < (n - baseSize); ++i) {
                x[baseSize + i] = m_data.m_optimizedRobotDofs(i);
            }
        }
        //We do not initialize the other components
        if (init_z) {
            return false;
        }
        if (init_lambda) {
            return false;
        }
        return true;
    }

    bool InverseKinematicsNLP::eval_f(Ipopt::Index n, const Ipopt::Number* x,
                                      bool new_x, Ipopt::Number& obj_value)
    {
        UNUSED_VARIABLE(n);
        if (new_x) {
            //First time we get called with this new value for the solution
            //Update the state and variables
            if (!updateState(x))
                return false;
        }

        //Cost function
        //J = Sum_i^#targets  Error on rotation   + ||q - q_des ||^2 (as regularization term)

        Eigen::Map<Eigen::VectorXd> qj_desired = iDynTree::toEigen(m_data.m_preferredJointsConfiguration);
        Eigen::Map<Eigen::VectorXd> jointError = iDynTree::toEigen(this->optimizedJoints);
        jointError -= qj_desired;

        obj_value = 0.5 * jointCostWeight * jointError.squaredNorm();

        if (m_data.m_targetResolutionMode != iDynTree::InverseKinematicsTreatTargetAsConstraintFull) {
            //if at least one cost mode
            //compute errors on rotation
            //TODO missing weights for the elements
            for (TransformMap::const_iterator target = m_data.m_targets.begin();
                 target != m_data.m_targets.end(); ++target) {

                if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                    && target->second.hasPositionConstraint()) {
                    //this implies that position is a soft constraint.
                    iDynTree::Position positionError = targetsInfo[target->first].transform.getPosition() - target->second.getPosition();
                    obj_value += 0.5 * (iDynTree::toEigen(positionError)).squaredNorm();
                }
                if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                    && target->second.hasRotationConstraint()) {
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
                    obj_value += 0.5 * (iDynTree::toEigen(orientationErrorQuaternion) - iDynTree::toEigen(identityQuaternion)).squaredNorm();
                }
            }
        }
        

        return true;
    }

    bool InverseKinematicsNLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                           Ipopt::Number* grad_f)
    {
        if (new_x) {
            //First time we get called with this new value for the solution
            //Update the state and variables
            if (!updateState(x))
                return false;
        }

        Eigen::Map<Eigen::VectorXd> gradient(grad_f, n);
        gradient.setZero();

        //First part of the gradient: the part of the cost depending only on q_j
        Eigen::Map<Eigen::VectorXd> qj = iDynTree::toEigen(this->optimizedJoints);

        Ipopt::Index baseSize = 3 + sizeOfRotationParametrization(m_data.m_rotationParametrization);
        //last n - baseSize elements are the ones corresponding to qj
        gradient.tail(n - baseSize) = jointCostWeight * (qj - iDynTree::toEigen(m_data.m_preferredJointsConfiguration)).transpose();

        //Second part of the gradient: this part depends on all q, i.e. x
        //compute errors on rotation
        if (m_data.m_targetResolutionMode != iDynTree::InverseKinematicsTreatTargetAsConstraintFull) {
            //if at least one cost mode
            for (TransformMap::const_iterator target = m_data.m_targets.begin();
                 target != m_data.m_targets.end(); ++target) {
                if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                    && target->second.hasPositionConstraint()) {
                    //this implies that position is a soft constraint.
                    iDynTree::Position positionError = targetsInfo[target->first].transform.getPosition() - target->second.getPosition();

                    computeConstraintJacobian(targetsInfo[target->first].jacobian,
                                              targetsInfo[target->first].quaternionDerivativeMap,
                                              quaternionDerivativeInverseMapBuffer,
                                              ComputeContraintJacobianOptionLinearPart,
                                              finalJacobianBuffer);

                    //Note: transpose probably creates a temporary matrix. As position is
                    //Fixedsize this should not fire a memory allocation in the heap, but we should
                    //profile the code at some point
                    gradient += iDynTree::toEigen(positionError).transpose() * iDynTree::toEigen(finalJacobianBuffer).topRows<3>();

                }
                if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                    && target->second.hasRotationConstraint()) {
                    //Derivative is (\tilde{Q} - 1) \partial_x Q
                    iDynTree::Rotation transformError = targetsInfo[target->first].transform.getRotation() * target->second.getRotation().inverse();

                    iDynTree::Vector4 orientationErrorQuaternion;
                    transformError.getQuaternion(orientationErrorQuaternion);

                    iDynTree::Vector4 identityQuaternion;
                    iDynTree::Rotation::Identity().getQuaternion(identityQuaternion);

                    computeConstraintJacobian(targetsInfo[target->first].jacobian,
                                              targetsInfo[target->first].quaternionDerivativeMap,
                                              quaternionDerivativeInverseMapBuffer,
                                              ComputeContraintJacobianOptionAngularPart,
                                              finalJacobianBuffer);

                    //These are the first baseSize columns + the joint columns
                    gradient += (iDynTree::toEigen(orientationErrorQuaternion) - iDynTree::toEigen(identityQuaternion)).transpose() * iDynTree::toEigen(finalJacobianBuffer).bottomRows<4>();
                }
            }
        }

        return true;
    }

    bool InverseKinematicsNLP::eval_g(Ipopt::Index n, const Ipopt::Number* x,
                                      bool new_x, Ipopt::Index m, Ipopt::Number* g)
    {
        UNUSED_VARIABLE(n);
        if (new_x) {
            //First time we get called with this new value for the solution
            //Update the state and variables
            if (!updateState(x))
                return false;
        }

        Ipopt::Index index = 0;
        Eigen::Map<Eigen::VectorXd> constraints(g, m);
        //Start with the explicit constraints
        for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
            constraint != m_data.m_constraints.end(); ++constraint) {
            iDynTree::Transform &currentTransform = constraintsInfo[constraint->first].transform;

            if (constraint->second.hasPositionConstraint()) {
                //get the position part
                const iDynTree::Position& currentPosition = currentTransform.getPosition();
                constraints.segment(index, 3) = iDynTree::toEigen(currentPosition);
                index += 3;
            }
            if (constraint->second.hasRotationConstraint()) {
                //get the rotation part
                const iDynTree::Rotation& currentRotation = currentTransform.getRotation();
                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

                    //quaternion parametrization
                    iDynTree::Vector4 quaternionBuffer;
                    currentRotation.getQuaternion(quaternionBuffer);
                    constraints.segment(index, 4) = iDynTree::toEigen(quaternionBuffer);
                } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {

                    currentRotation.getRPY(constraints(index), constraints(index + 1), constraints(index + 2));
                }
                index += sizeOfRotationParametrization(m_data.m_rotationParametrization);
            }
        }

        //Targets considered as constraints
        for (TransformMap::const_iterator target = m_data.m_targets.begin();
             target != m_data.m_targets.end(); ++target) {
            iDynTree::Transform &currentTransform = targetsInfo[target->first].transform;

            if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                && target->second.hasPositionConstraint()) {
                //add the position target as constraint
                const iDynTree::Position& currentPosition = currentTransform.getPosition();
                constraints.segment(index, 3) = iDynTree::toEigen(currentPosition);
                index += 3;
            }
            if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
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
            constraints[index++] = iDynTree::toEigen(this->baseOrientation).squaredNorm();
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

            //As we do not have sparse jacobians for now
            //the jacobian has the same number of rows as the constraints
            Ipopt::Index jacobianRows = m;
            //The only exception is the base expressed in quaternion which is sparse
            //So we remove one line as we add it back after the dense part
            if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                jacobianRows--;
            }
            //Fill sparsity structure of the jacobian
            //For now the jacobian is dense, so I don't care of the pattern
            for (Ipopt::Index row = 0; row < jacobianRows; row++) {
                for (Ipopt::Index col = 0; col < n; col++) {
                    //row-based indexing
                    iRow[row * n + col] = row;
                    jCol[row * n + col] = col;
                    index++;
                }
            }

            //If the base orientation is parametrized in quaternion
            //add the sparsity of the jacobian for that constraint
            if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                iRow[index] = m - 1;
                jCol[index] = 3;
                index++;
                iRow[index] = m - 1;
                jCol[index] = 4;
                index++;
                iRow[index] = m - 1;
                jCol[index] = 5;
                index++;
                iRow[index] = m - 1;
                jCol[index] = 6;
                index++;
            }

            assert(nele_jac == index);
            
        } else {
            //This is called every time
            if (new_x) {
                //First time we get called with this new value for the solution
                //Update the state and variables
                if (!updateState(x))
                    return false;
            }

            Ipopt::Index constraintIndex = 0;

            for (TransformMap::const_iterator constraint = m_data.m_constraints.begin();
                 constraint != m_data.m_constraints.end(); ++constraint) {
                //For each constraint compute its jacobian
                FrameInfo &constraintInfo = constraintsInfo[constraint->first];

                //Get the Frame Jacobian
                iDynTree::iDynTreeEigenMatrixMap constraintJacobian = iDynTree::toEigen(constraintInfo.jacobian);

                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

                    //We adapt the jacobian to handle two things
                    // - the orientation part is represented as a quaternion
                    // - the iDynTree Jacobian gives the corresponding velocity in term of
                    //   angular velocity. Here the derivative is not w.r.t. time
                    //   but w.r.t. an arbitrary variation of the configuration
                    computeConstraintJacobian(constraintInfo.jacobian,
                                              constraintInfo.quaternionDerivativeMap,
                                              quaternionDerivativeInverseMapBuffer,
                                              ComputeContraintJacobianOptionLinearPart|ComputeContraintJacobianOptionAngularPart,
                                              finalJacobianBuffer);

                    //The Eigen map now points to the modified Jacobian (7 x 7 + ndofs)
                    new (&constraintJacobian) iDynTree::iDynTreeEigenMatrixMap(finalJacobianBuffer.data(), finalJacobianBuffer.rows(), finalJacobianBuffer.cols());

                } else if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw) {
                    //RPY parametrization
                    iDynTree::Matrix3x3 omegaToRPYMap;
                    iDynTree::Vector3 rpy;
                    iDynTree::toEigen(rpy) = iDynTree::toEigen(this->baseOrientation).head<3>();
                    omegaToRPYParameters(rpy, omegaToRPYMap);
                    constraintJacobian.bottomRows<3>() = iDynTree::toEigen(omegaToRPYMap) * constraintJacobian.bottomRows<3>();
                }

                //Now that we computed the actual Jacobian needed by IPOPT
                //We have to assign it to the correct variable
                if (constraint->second.hasPositionConstraint()) {
                    //Position part
                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > currentConstraint(&values[constraintIndex], 3, n);
                    currentConstraint = constraintJacobian.topRows<3>();
                    constraintIndex += 3;
                }
                if (constraint->second.hasRotationConstraint()) {
                    //Orientation part
                    for (Ipopt::Index row = 0; row < sizeOfRotationParametrization(m_data.m_rotationParametrization); ++row) {

                        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > currentConstraint(&values[constraintIndex], sizeOfRotationParametrization(m_data.m_rotationParametrization), n);
                        currentConstraint = constraintJacobian.bottomRows(sizeOfRotationParametrization(m_data.m_rotationParametrization));
                        constraintIndex += sizeOfRotationParametrization(m_data.m_rotationParametrization);
                    }
                    constraintIndex += sizeOfRotationParametrization(m_data.m_rotationParametrization);
                }
            }

            //For all targets enforced as constraints
            for (TransformMap::const_iterator target = m_data.m_targets.begin();
                 target != m_data.m_targets.end(); ++target) {

                FrameInfo &targetInfo = targetsInfo[target->first];

                iDynTree::iDynTreeEigenMatrixMap constraintJacobian = iDynTree::toEigen(targetInfo.jacobian);

                if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {

                    //Depending if we need position and/or orientation
                    //we have to adapt different parts of the jacobian
                    int computationOption = 0;
                    if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly)
                        computationOption |= ComputeContraintJacobianOptionLinearPart;
                    if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly)
                        computationOption |= ComputeContraintJacobianOptionAngularPart;

                    computeConstraintJacobian(targetInfo.jacobian,
                                              targetInfo.quaternionDerivativeMap,
                                              quaternionDerivativeInverseMapBuffer,
                                              computationOption,
                                              finalJacobianBuffer);

                    //Modify the Eigen Map to point to the Jacobain of size (7 x 7 + ndofs)
                    new (&constraintJacobian) iDynTree::iDynTreeEigenMatrixMap(finalJacobianBuffer.data(),
                                                                               finalJacobianBuffer.rows(),
                                                                               finalJacobianBuffer.cols());
                }

                if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintPositionOnly
                    && target->second.hasPositionConstraint()) {

                    //Copy position part
                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > currentConstraint(&values[constraintIndex], 3, n);
                    currentConstraint = constraintJacobian.topRows<3>();
                    constraintIndex += 3;
                }

                if (m_data.m_targetResolutionMode & iDynTree::InverseKinematicsTreatTargetAsConstraintRotationOnly
                    && target->second.hasRotationConstraint()) {
                    //Orientation part

                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > currentConstraint(&values[constraintIndex], sizeOfRotationParametrization(m_data.m_rotationParametrization), n);
                    currentConstraint = constraintJacobian.bottomRows(sizeOfRotationParametrization(m_data.m_rotationParametrization));
                    constraintIndex += sizeOfRotationParametrization(m_data.m_rotationParametrization);
                }
            }

            //Finally, the norm of the base orientation quaternion parametrization
            if (m_data.m_rotationParametrization == iDynTree::InverseKinematicsRotationParametrizationQuaternion) {
                //Quaternion norm derivative
                // = 2 * Q^\top
                Eigen::Map<Eigen::VectorXd> quaternionDerivative(&values[constraintIndex * n], 4);
                quaternionDerivative = 2 * iDynTree::toEigen(this->baseOrientation).transpose();
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
        m_data.m_optimizedBasePosition(0) = x[0];
        m_data.m_optimizedBasePosition(1) = x[1];
        m_data.m_optimizedBasePosition(2) = x[2];


        //TODO: check status

        //orientation
        for (unsigned i = 0; i < sizeOfRotationParametrization(m_data.m_rotationParametrization); ++i) {
            m_data.m_optimizedBaseOrientation(i) = x[3 + i];
        }

        //joints
        for (unsigned index = 0; index < m_data.m_dofs; ++index) {
            m_data.m_optimizedRobotDofs(index) = x[3 + sizeOfRotationParametrization(m_data.m_rotationParametrization) + index];
        }


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
        map(2, 0) = -std::cos(rpyAngles(2))* std::tan(rpyAngles(1));
        map(2, 1) = -std::sin(rpyAngles(2))* std::tan(rpyAngles(1));
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
        Map<Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > analyticalJacobian = toEigen(_analyticalJacobian);

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
        Map<Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > finiteDifferenceJacobian = toEigen(_finiteDifferenceJacobian);

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
        for (int i = 0; i < derivativePoint.size(); ++i) {
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
