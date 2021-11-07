#include "idyntree_high_level.h"
#include "idyntree_type_caster.h"

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>

namespace iDynTree
{
namespace bindings
{
namespace
{
namespace py = ::pybind11;

struct KinDynKinematicsRobotState
{
    iDynTree::VectorDynSize jointPositions;
    iDynTree::VectorDynSize jointVelocities;

    iDynTree::Twist baseVelocity;
    iDynTree::Transform baseTransform;

    iDynTree::Vector3 worldGravity;
};

} // namespace
void iDynTreeHighLevelBindings(pybind11::module& module)
{
    py::class_<KinDynKinematicsRobotState>(module, "KinDynKinematicsRobotState")
        .def(py::init())
        .def_readwrite("joint_positions", &KinDynKinematicsRobotState::jointPositions)
        .def_readwrite("joint_velocities", &KinDynKinematicsRobotState::jointVelocities)
        .def_readwrite("base_velocity", &KinDynKinematicsRobotState::baseVelocity)
        .def_readwrite("base_transform", &KinDynKinematicsRobotState::baseTransform)
        .def_readwrite("world_gravity", &KinDynKinematicsRobotState::worldGravity);

    py::class_<KinDynComputations, std::shared_ptr<KinDynComputations>>(module,
                                                                        "KinDynComputations")
        .def(py::init())
        .def("load_robot_model", &KinDynComputations::loadRobotModel, py::arg("model"))
        .def("is_valid", &KinDynComputations::isValid)
        .def("get_nr_of_degrees_of_freedom", &KinDynComputations::getNrOfDegreesOfFreedom)
        .def("get_description_of_degree_of_freedom",
             &KinDynComputations::getDescriptionOfDegreeOfFreedom)
        .def("get_description_of_degrees_of_freedom",
             &KinDynComputations::getDescriptionOfDegreesOfFreedom)
        .def("get_nr_of_links", &KinDynComputations::getNrOfLinks)
        .def("get_nr_of_frames", &KinDynComputations::getNrOfFrames)
        .def("get_floating_base", &KinDynComputations::getFloatingBase)
        .def("set_floating_base",
             &KinDynComputations::setFloatingBase,
             py::arg("floating_base_name"))
        .def_property("floating_base",
                      &KinDynComputations::getFloatingBase,
                      &KinDynComputations::setFloatingBase)
        .def("get_robot_model", &KinDynComputations::getRobotModel)
        .def_property("model",
                      &KinDynComputations::getRobotModel,
                      &KinDynComputations::loadRobotModel)
        .def("get_relative_jacobian_sparsity_pattern",
             [](const KinDynComputations& impl,
                const iDynTree::FrameIndex refFrameIndex,
                const iDynTree::FrameIndex frameIndex) -> iDynTree::MatrixDynSize {
                 iDynTree::MatrixDynSize jacobian;
                 if (!impl.getRelativeJacobianSparsityPattern(refFrameIndex, frameIndex, jacobian))
                 {
                     throw py::value_error("File to get the relative jacobian sparsity pattern.");
                 }
                 return jacobian;
             })
        .def("get_frame_free_floating_jacobian_sparsity_pattern",
             [](const KinDynComputations& impl,
                const iDynTree::FrameIndex frameIndex) -> iDynTree::MatrixDynSize {
                 iDynTree::MatrixDynSize jacobian;
                 if (!impl.getFrameFreeFloatingJacobianSparsityPattern(frameIndex, jacobian))
                 {
                     throw py::value_error("File to get the free floating jacobian sparsity "
                                           "pattern.");
                 }
                 return jacobian;
             })
        .def("set_joint_pos",
             py::overload_cast<const iDynTree::VectorDynSize&>(&KinDynComputations::setJointPos),
             py::arg("s"))
        .def("set_robot_state",
             py::overload_cast<const iDynTree::Transform&,
                               const iDynTree::VectorDynSize&,
                               const iDynTree::Twist&,
                               const iDynTree::VectorDynSize&,
                               const iDynTree::Vector3&>(&KinDynComputations::setRobotState),
             py::arg("world_T_base"),
             py::arg("s"),
             py::arg("base_velocity"),
             py::arg("s_dot"),
             py::arg("gravity"))
        .def("set_robot_state",
             py::overload_cast<const iDynTree::VectorDynSize&,
                               const iDynTree::VectorDynSize&,
                               const iDynTree::Vector3&>(&KinDynComputations::setRobotState),
             py::arg("s"),
             py::arg("s_dot"),
             py::arg("gravity"))
        .def("get_robot_state",
             [](KinDynComputations& impl) -> KinDynKinematicsRobotState {
                 KinDynKinematicsRobotState robotState;
                 impl.getRobotState(robotState.baseTransform,
                                    robotState.jointPositions,
                                    robotState.baseVelocity,
                                    robotState.jointVelocities,
                                    robotState.worldGravity);
                 return robotState;
             })
        .def("get_world_base_transform",
             py::overload_cast<>(&KinDynComputations::getWorldBaseTransform, py::const_))
        .def("get_base_twist", py::overload_cast<>(&KinDynComputations::getBaseTwist, py::const_))
        .def("get_joint_pos",
             [](const iDynTree::KinDynComputations& impl) {
                 VectorDynSize s;
                 if (!impl.getJointPos(s))
                 {
                     throw py::value_error("Failed to get joint position.");
                 }
                 return s;
             })
        .def("get_joint_vel",
             [](const iDynTree::KinDynComputations& impl) {
                 VectorDynSize ds;
                 if (!impl.getJointVel(ds))
                 {
                     throw py::value_error("Failed to get joint velocity.");
                 }
                 return ds;
             })
        .def("get_model_vel",
             [](const iDynTree::KinDynComputations& impl) {
                 VectorDynSize nu;
                 if (!impl.getModelVel(nu))
                 {
                     throw py::value_error("Failed to get model velocity.");
                 }
                 return nu;
             })
        .def_property_readonly("world_base_transform",
                               py::overload_cast<>(&KinDynComputations::getWorldBaseTransform,
                                                   py::const_))
        .def_property_readonly("base_twist",
                               py::overload_cast<>(&KinDynComputations::getBaseTwist, py::const_))
        .def_property_readonly("joint_pos",
                               [](const iDynTree::KinDynComputations& impl) {
                                   VectorDynSize s;
                                   if (!impl.getJointPos(s))
                                   {
                                       throw py::value_error("Failed to get joint position.");
                                   }
                                   return s;
                               })
        .def_property_readonly("joint_vel",
                               [](const iDynTree::KinDynComputations& impl) {
                                   VectorDynSize ds;
                                   if (!impl.getJointVel(ds))
                                   {
                                       throw py::value_error("Failed to get joint velocity.");
                                   }
                                   return ds;
                               })
        .def_property_readonly("get_model_vel",
                               [](const iDynTree::KinDynComputations& impl) {
                                   VectorDynSize nu;
                                   if (!impl.getModelVel(nu))
                                   {
                                       throw py::value_error("Failed to get model velocity.");
                                   }
                                   return nu;
                               })
        .def("get_frame_index", &KinDynComputations::getFrameIndex, py::arg("frame_name"))
        .def("get_frame_name", &KinDynComputations::getFrameName, py::arg("frame_index"))
        .def("get_world_transform",
             py::overload_cast<const iDynTree::FrameIndex>(&KinDynComputations::getWorldTransform),
             py::arg("frame_index"))
        .def("get_world_transform",
             py::overload_cast<const std::string&>(&KinDynComputations::getWorldTransform),
             py::arg("frame_name"))
        .def("get_world_transforms_as_homogeneous",
             &KinDynComputations::getWorldTransformsAsHomogeneous,
             py::arg("frame_names"))
        .def("get_relative_transform",
             py::overload_cast<const iDynTree::FrameIndex, const iDynTree::FrameIndex>(
                 &KinDynComputations::getRelativeTransform),
             py::arg("ref_frame_index"),
             py::arg("frame_index"))
        .def("get_relative_transform_explcit",
             py::overload_cast<const iDynTree::FrameIndex,
                               const iDynTree::FrameIndex,
                               const iDynTree::FrameIndex,
                               const iDynTree::FrameIndex>(
                 &KinDynComputations::getRelativeTransformExplicit),
             py::arg("ref_frame_origin_index"),
             py::arg("ref_frame_orientation_index"),
             py::arg("frame_origin_index"),
             py::arg("frame_orientation_index"))
        .def("get_relative_transform",
             py::overload_cast<const std::string&, const std::string&>(
                 &KinDynComputations::getRelativeTransform),
             py::arg("ref_frame_name"),
             py::arg("frame_name"))
        .def("get_frame_vel",
             py::overload_cast<const std::string&>(&KinDynComputations::getFrameVel),
             py::arg("frame_name"))
        .def("get_frame_vel",
             py::overload_cast<const FrameIndex>(&KinDynComputations::getFrameVel),
             py::arg("frame_index"))
        .def("get_frame_acc",
             py::overload_cast<const FrameIndex,
                               const iDynTree::Vector6&,
                               const iDynTree::VectorDynSize&>(&KinDynComputations::getFrameAcc),
             py::arg("frame_index"),
             py::arg("base_acc"),
             py::arg("s_ddot"))
        .def("get_frame_acc",
             py::overload_cast<const std::string&,
                               const iDynTree::Vector6&,
                               const iDynTree::VectorDynSize&>(&KinDynComputations::getFrameAcc),
             py::arg("frame_name"),
             py::arg("base_acc"),
             py::arg("s_ddot"))
        .def(
            "get_frame_free_floating_jacobian",
            [](KinDynComputations& impl, const std::string& frameName) {
                MatrixDynSize jacobian;
                if (!impl.getFrameFreeFloatingJacobian(frameName, jacobian))
                {
                    throw py::value_error("Unable to get the free floating jacobian.");
                }
                return jacobian;
            },
            py::arg("frame_name"))
        .def(
            "get_frame_free_floating_jacobian",
            [](KinDynComputations& impl, const FrameIndex frameIndex) {
                MatrixDynSize jacobian;
                if (!impl.getFrameFreeFloatingJacobian(frameIndex, jacobian))
                {
                    throw py::value_error("Unable to get the free floating jacobian.");
                }
                return jacobian;
            },
            py::arg("frame_index"))
        .def(
            "get_relative_jacobian",
            [](KinDynComputations& impl,
               const iDynTree::FrameIndex refFrameIndex,
               const iDynTree::FrameIndex frameIndex) {
                MatrixDynSize jacobian;
                if (!impl.getRelativeJacobian(refFrameIndex, frameIndex, jacobian))
                {
                    throw py::value_error("Unable to get the relative jacobian.");
                }
                return jacobian;
            },
            py::arg("ref_frame_index"),
            py::arg("frame_index"))
        .def(
            "get_relative_jacobian_explicit ",
            [](KinDynComputations& impl,
               const iDynTree::FrameIndex refFrameIndex,
               const iDynTree::FrameIndex frameIndex,
               const iDynTree::FrameIndex expressedOriginFrameIndex,
               const iDynTree::FrameIndex expressedOrientationFrameIndex) {
                MatrixDynSize jacobian;
                if (!impl.getRelativeJacobianExplicit(refFrameIndex,
                                                      frameIndex,
                                                      expressedOriginFrameIndex,
                                                      expressedOrientationFrameIndex,
                                                      jacobian))
                {
                    throw py::value_error("Unable to get the explicit relative jacobian.");
                }
                return jacobian;
            },
            py::arg("ref_frame_index"),
            py::arg("frame_index"),
            py::arg("expressed_origin_frame_index"),
            py::arg("expressed_orientation_frame_index"))
        .def("get_frame_bias_acc",
             py::overload_cast<const std::string&>(&KinDynComputations::getFrameBiasAcc),
             py::arg("frame_name"))
        .def("get_frame_bias_acc",
             py::overload_cast<const FrameIndex>(&KinDynComputations::getFrameBiasAcc),
             py::arg("frame_index"))
        .def("get_center_of_mass_position",
             py::overload_cast<>(&KinDynComputations::getCenterOfMassPosition))
        .def("get_center_of_mass_velocity",
             py::overload_cast<>(&KinDynComputations::getCenterOfMassVelocity))
        .def("get_center_of_mass_jacobian",
             [](KinDynComputations& impl) {
                 MatrixDynSize jacobian;
                 if (!impl.getCenterOfMassJacobian(jacobian))
                 {
                     throw py::value_error("Unable to get the center of mass jacobian.");
                 }
                 return jacobian;
             })
        .def("get_center_of_mass_bias_acc",
             py::overload_cast<>(&KinDynComputations::getCenterOfMassBiasAcc))
        .def("get_average_velocity", py::overload_cast<>(&KinDynComputations::getAverageVelocity))
        .def("get_average_velocity_jacobian",
             [](KinDynComputations& impl) {
                 MatrixDynSize jacobian;
                 if (!impl.getAverageVelocityJacobian(jacobian))
                 {
                     throw py::value_error("Unable to get the center of mass jacobian.");
                 }
                 return jacobian;
             })
        .def("get_centroidal_average_velocity",
             py::overload_cast<>(&KinDynComputations::getCentroidalAverageVelocity))
        .def("get_centroidal_average_velocity_jacobian",
             [](KinDynComputations& impl) {
                 MatrixDynSize jacobian;
                 if (!impl.getCentroidalAverageVelocityJacobian(jacobian))
                 {
                     throw py::value_error("Unable to get the center of mass jacobian.");
                 }
                 return jacobian;
             })
        .def("get_linear_angular_momentum",
             py::overload_cast<>(&KinDynComputations::getLinearAngularMomentum))
        .def("get_linear_angular_momentum_jacobian",
             [](KinDynComputations& impl) {
                 MatrixDynSize jacobian;
                 if (!impl.getLinearAngularMomentumJacobian(jacobian))
                 {
                     throw py::value_error("Unable to get linear and angular momentum jacobian.");
                 }
                 return jacobian;
             })
        .def("get_centroidal_total_momentum",
             py::overload_cast<>(&KinDynComputations::getCentroidalTotalMomentum))
        .def("get_centroidal_total_momentum_jacobian",
             [](KinDynComputations& impl) {
                 MatrixDynSize jacobian;
                 if (!impl.getCentroidalTotalMomentumJacobian(jacobian))
                 {
                     throw py::value_error("Unable to get centroidal momentum jacobian.");
                 }
                 return jacobian;
             })
        .def("get_free_floating_mass_matrix",
             [](KinDynComputations& impl) {
                 MatrixDynSize massMatrix;
                 if (!impl.getFreeFloatingMassMatrix(massMatrix))
                 {
                     throw py::value_error("Unable to get the mass matrix.");
                 }
                 return massMatrix;
             })
        .def("generalized_bias_forces",
             [](KinDynComputations& impl) {
                 FreeFloatingGeneralizedTorques biasForces(impl.model());
                 if (!impl.generalizedBiasForces(biasForces))
                 {
                     throw py::value_error("Unable to get the bias forces.");
                 }
                 return biasForces;
             })
        .def("generalizedGravityForces", [](KinDynComputations& impl) {
            FreeFloatingGeneralizedTorques gravitationalForces(impl.model());
            if (!impl.generalizedGravityForces(gravitationalForces))
            {
                throw py::value_error("Unable to get the gravitational forces.");
            }
            return gravitationalForces;
        });
}
} // namespace bindings
} // namespace iDynTree
