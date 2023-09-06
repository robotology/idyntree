// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H
#define IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/SparsityStructure.h>
#include <vector>
#include <string>
#include <memory>
#include <map>

namespace iDynTree {

namespace optimalcontrol {

        class DynamicalSystem;

        namespace integrators {

            /**
             * @warning This class is still in active development, and so API interface can change between iDynTree versions.
             * \ingroup iDynTreeExperimental
             */

            /**
             * @brief Output of the integrator at a specific time.
             */

            class SolutionElement{
            public:
                /**
                 * @brief State value
                 */
                VectorDynSize stateAtT;
                /**
                 * @brief Time at which stateAtT is computed.
                 */
                double time;
            };

            class IntegratorInfoData {
            protected:
                friend class Integrator;
                IntegratorInfoData();
            public:
                std::string name;

                bool isExplicit;

                size_t numberOfStages;
            };

            /**
             * @warning This class is still in active development, and so API interface can change between iDynTree versions.
             * \ingroup iDynTreeExperimental
             */

            /**
             * @brief Class containing infos about the implemented integrator.
             *
             * This class provides a read-only access to the IntegratorInfoData class. An instance of this class is created by the base class Integrator.
             * All integrators inheriting from this base class should fill in such informations.
             */
            class IntegratorInfo {
            private:
                std::shared_ptr<IntegratorInfoData> m_data;
            public:
                IntegratorInfo(std::shared_ptr<IntegratorInfoData> data);

                IntegratorInfo() = delete;

                IntegratorInfo(const IntegratorInfo &other) = delete;

                /**
                 * @brief Name of the integration method.
                 */
                const std::string &name() const;

                /**
                 * @brief Tells if the integration method is explicit or not
                 */
                bool isExplicit() const;

                /**
                 * @brief Number of stages of the integration method.
                 *
                 * Currently it is not used in th library.
                 * @return The number of stages of the multistage method.
                 */
                size_t numberOfStages() const;
            };


            class CollocationHessianIndex {
                size_t m_first;
                size_t m_second;

            public:

                CollocationHessianIndex() = delete;

                CollocationHessianIndex(size_t first, size_t second);

                bool operator< (const CollocationHessianIndex& rhs) const;

                size_t first() const;

                size_t second() const;
            };

            using CollocationHessianMap = std::map<CollocationHessianIndex, MatrixDynSize>;

            using CollocationHessianSparsityMap = std::map<CollocationHessianIndex, SparsityStructure>;

            /**
             * @warning This class is still in active development, and so API interface can change between iDynTree versions.
             * \ingroup iDynTreeExperimental
             */

            /**
             * @brief The Integrator base class.
             *
             * Inherit publicly from this class in order to specify a custom integration method.
             */
            class Integrator {

            public:

                /**
                 * @brief Integrator default constructor.
                 *
                 * This constructor allows to create an Integrator object without specifying a DynamicalSystem yet. This has to be done via the method setDynamicalSystem before calling the
                 * integrate method.
                 */
                Integrator();

                /**
                 * @brief Integrator constructor.
                 * @param[in] dynamicalSystem Pointer to the DynamicalSystem to be integrated.
                 */
                Integrator(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                Integrator(const Integrator &other) = delete;

                virtual ~Integrator();

                /**
                 * @brief Integrate the dynamical system with specified integration bounds.
                 *
                 * This method should be implemented by the custom integrator.
                 * @param[in] initialTime Lower integration bound.
                 * @param[in] finalTime Upper integration bound.
                 * @return true if successfull.
                 */
                virtual bool integrate(double initialTime, double finalTime) = 0;

                /**
                 * @brief Specify the maximum integration step size.
                 *
                 * This parameter is considered when integrating a DynamicalSystem.
                 * @param[in] dT The masimum step size.
                 * @return true if successfull, false otherwise (for example if dT is not strictly positive)
                 */
                bool setMaximumStepSize(const double dT);

                /**
                 * @brief Return the maximum step size set with setMaximumStepSize.
                 */
                double maximumStepSize() const;

                /**
                 * @brief Set the DynamicalSystem to be considered.
                 *
                 * This methods changes the dynamical system only if it was not already set.
                 * @param[in] dynamicalSystem The dynamical system pointer
                 * @return true if successfull, false otherwise, for example if a DynamicalSystem was already set.
                 */
                bool setDynamicalSystem(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                /**
                 * @brief Weak pointer to the specified DynamicalSystem.
                 */
                const std::weak_ptr<DynamicalSystem> dynamicalSystem() const;

                /**
                 * @brief Retrieve integration solution at a specified time.
                 *
                 * The method integrate should have been called first, and time should be within the integration bounds, otherwise returns false.
                 *
                 * It sould not be necessary to override this method.
                 * @param[in] time Instant of interest.
                 * @param solution State corresponding to the specified time.
                 * @return true if successfull.
                 */
                virtual bool getSolution(double time, VectorDynSize& solution) const;

                /**
                 * @brief Retrieve the full buffer of SolutionElement
                 *
                 * The method integrate should have been called first, and time should be within the integration bounds, otherwise returns false.
                 *
                 * It sould not be necessary to override this method.
                 * @return A const reference to the internal buffer contaning the full output of the integration routine.
                 */
                virtual const std::vector<SolutionElement>& getFullSolution() const;

                /**
                 * @brief Clear the solution buffer.
                 */
                virtual void clearSolution();


                /**
                 * @brief Evaluate the collocation constraint.
                 *
                 * In some optimal control solvers, like the MultipleShootingSolver, we need to discretize the dynamical system associated to the optimal control problem.
                 * This function evaluates the discretization error according to specified integration method. The Integrator, when integrating a dynamical system, is performing the following discretization:
                 * \f[
                 * x_{k+1} = \phi(x_k, x_{k+1}, u_k, u_{k+1}, t)
                 * \f]
                 * where \f$ x_{k+1}\f$ is the discretized output. For example, using forward euler, we would have
                 * \f[
                 * x_{k+1} = x_k + f(t, x, u)\mathrm{d}T.
                 * \f]
                 *
                 * This function returns the result of \f$ \phi(x_k, x_{k+1}, u_k, u_{k+1}, t) - x_{k+1}\f$.
                 *
                 * @param[in] time Instant at which the collocation constraint is evaluated
                 * @param[in] collocationPoints A vector containing \f$x_k\f$ and \f$x_{k+1}\f$. Notice that \f$x_{k+1}\f$ corresponds to the state at instant (time + dT).
                 * @param[in] controlInputs A vector containing \f$u_k\f$ and \f$u_{k+1}\f$. Notice that \f$u_{k+1}\f$ is supposed to be applied at instant (time + dT).
                 * @param[in] dT The time distance between \f$x_k\f$ and \f$x_{k+1}\f$.
                 * @param[out] constraintValue The result of \f$ \phi(x_k, x_{k+1}, u_k, u_{k+1}, t) - x_{k+1}\f$.
                 * @return True if successfull, false otherwise (or if not implemented).
                 */
                virtual bool evaluateCollocationConstraint(double time, const std::vector<VectorDynSize>& collocationPoints,
                                                           const std::vector<VectorDynSize>& controlInputs, double dT,
                                                           VectorDynSize& constraintValue);

                virtual bool evaluateCollocationConstraintJacobian(double time, const std::vector<VectorDynSize>& collocationPoints,
                                                                   const std::vector<VectorDynSize>& controlInputs, double dT,
                                                                   std::vector<MatrixDynSize>& stateJacobianValues,
                                                                   std::vector<MatrixDynSize>& controlJacobianValues);

                virtual bool getCollocationConstraintJacobianStateSparsity(std::vector<SparsityStructure>& stateJacobianSparsity);

                virtual bool getCollocationConstraintJacobianControlSparsity(std::vector<SparsityStructure>& controlJacobianSparsity);


                virtual bool evaluateCollocationConstraintSecondDerivatives(double time, const std::vector<VectorDynSize>& collocationPoints,
                                                                            const std::vector<VectorDynSize>& controlInputs, double dT,
                                                                            const VectorDynSize& lambda,
                                                                            CollocationHessianMap& stateSecondDerivative,
                                                                            CollocationHessianMap& controlSecondDerivative,
                                                                            CollocationHessianMap& stateControlSecondDerivative);

                virtual bool getCollocationConstraintSecondDerivativeWRTStateSparsity(CollocationHessianSparsityMap& stateDerivativeSparsity);

                virtual bool getCollocationConstraintSecondDerivativeWRTControlSparsity(CollocationHessianSparsityMap& controlDerivativeSparsity);

                virtual bool getCollocationConstraintSecondDerivativeWRTStateControlSparsity(CollocationHessianSparsityMap& stateControlDerivativeSparsity);

                const IntegratorInfo& info() const;

            protected:
                double m_dTmax;
                std::shared_ptr<DynamicalSystem> m_dynamicalSystem_ptr;
                std::vector<SolutionElement> m_solution;
                std::shared_ptr<IntegratorInfoData> m_infoData;
                IntegratorInfo m_info;

                virtual bool interpolatePoints(const std::vector<SolutionElement>::const_iterator &first,
                                               const std::vector<SolutionElement>::const_iterator &second,
                                               double time, VectorDynSize& outputPoint) const;

                virtual bool allocateBuffers();
            };
        }

    }
}



#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H */
