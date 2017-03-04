/*!
 * @file InverseKinematicsNLP.h
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#ifndef IDYNTREE_INTERNAL_INVERSEKINEMATICSNLP_H
#define IDYNTREE_INTERNAL_INVERSEKINEMATICSNLP_H

#include <IpTNLP.hpp>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/Transform.h>

#include <map> //if moving to c++11 use unordered map here which is faster

// use expression as sub-expression,
// then make type of full expression int, discard result
#define UNUSED_VARIABLE(x) (void)(sizeof((x), 0))

namespace internal {
namespace kinematics {
    class InverseKinematicsNLP;
    class InverseKinematicsData;
}
}

class internal::kinematics::InverseKinematicsNLP : public Ipopt::TNLP {

    struct FrameInfo {
        iDynTree::Transform transform;
        iDynTree::MatrixDynSize jacobian;
        iDynTree::MatrixFixSize<4, 3> quaternionDerivativeMap;
    };
    typedef std::map<int, FrameInfo> FrameInfoMap;

    InverseKinematicsData& m_data;
    iDynTree::VectorDynSize jointsConfiguration; //this is used to update the model at an optimization step

    //Buffers and variables used in the optimization
    iDynTree::MatrixDynSize transformWithQuaternionJacobianBuffer;
    iDynTree::MatrixFixSize<4, 3> quaternionDerivativeMapBuffer;
    iDynTree::MatrixFixSize<3, 4> quaternionDerivativeInverseMapBuffer;
    iDynTree::MatrixDynSize finalJacobianBuffer;

    FrameInfoMap constraintsInfo;
    FrameInfoMap targetsInfo;

    //Temporary optimized variables
    iDynTree::Position basePosition;
    iDynTree::Vector4 baseOrientation;
    iDynTree::VectorDynSize optimizedJoints;

    double jointCostWeight;

    bool updateState(const Ipopt::Number * x);

    void initializeInternalData(Ipopt::Index n, Ipopt::Index m);

    enum ComputeContraintJacobianOption {
        ComputeContraintJacobianOptionLinearPart = 1,
        ComputeContraintJacobianOptionAngularPart = 1 << 1,
    };

    void computeConstraintJacobian(const iDynTree::MatrixDynSize& transformJacobian,
                                   const iDynTree::MatrixFixSize<4, 3>& quaternionDerivativeMapBuffer,
                                   const iDynTree::MatrixFixSize<3, 4>& quaternionDerivativeInverseMapBuffer,
                                   const int computationOption,
                                   iDynTree::MatrixDynSize& constraintJacobianBuffer);

    void omegaToRPYParameters(const iDynTree::Vector3& rpyAngles,
                              iDynTree::Matrix3x3& map);

public:
    InverseKinematicsNLP(InverseKinematicsData& data);

#pragma mark - IpOpt methods

    virtual ~InverseKinematicsNLP();

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda);

    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Number& obj_value);

    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                             Ipopt::Number* grad_f);

    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Index m, Ipopt::Number* g);

    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                            Ipopt::Index *jCol, Ipopt::Number* values);

    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values);

    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                   const Ipopt::Number* x, const Ipopt::Number* z_L,
                                   const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                   const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq);

    virtual Ipopt::Index get_number_of_nonlinear_variables();

    virtual bool get_list_of_nonlinear_variables(Ipopt::Index num_nonlin_vars,
                                                 Ipopt::Index* pos_nonlin_vars);

    void testDerivatives(const iDynTree::VectorDynSize& derivativePoint, int frameIndex, double epsilon, double tolerance, int parametrization);

    virtual bool intermediate_callback(Ipopt::AlgorithmMode mode,
                                                             Ipopt::Index iter, Ipopt::Number obj_value,
                                                             Ipopt::Number inf_pr, Ipopt::Number inf_du,
                                                             Ipopt::Number mu, Ipopt::Number d_norm,
                                                             Ipopt::Number regularization_size,
                                                             Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                                                             Ipopt::Index ls_trials,
                                                             const Ipopt::IpoptData* ip_data,
                                                             Ipopt::IpoptCalculatedQuantities* ip_cq);
};

#endif /* end of include guard: IDYNTREE_INTERNAL_INVERSEKINEMATICSNLP_H */
