
/* File : iDynTree.i */
%module iDynTree
#ifdef SWIGPYTHON
%implicitconv;
#endif

%include "std_string.i"
%include "std_vector.i"

// std::shared_ptr holder is currently supported only for python bindings
#ifdef SWIGPYTHON
%include "std_shared_ptr.i"
#endif

// Wrap the std::vector<std::string> params
%template(StringVector) std::vector<std::string>;

// Wrap the std::vector<std::int> params
%template(IntVector) std::vector<int>;

// Ignore some methods to avoid warnings
%include "./ignore.i"


// Python
#ifdef SWIGPYTHON
%include "./python/python.i"
#endif

// Matlab
#ifdef SWIGMATLAB
%include "./matlab/matlab.i"
#endif

%{
/* Note : always include headers following the inheritance order */
#include <cmath>

//Utils
#include "iDynTree/Utils.h"

// Basic math classes
#include "iDynTree/MatrixDynSize.h"
#include "iDynTree/MatrixFixSize.h"
#include "iDynTree/SparseMatrix.h"

#include "iDynTree/VectorDynSize.h"
#include "iDynTree/VectorFixSize.h"

// Basic Vectors: Point Vectors and Spatial Vectors
#include "iDynTree/PositionRaw.h"
#include "iDynTree/Position.h"
#include "iDynTree/SpatialForceVector.h"
#include "iDynTree/SpatialMotionVector.h"
#include "iDynTree/Twist.h"
#include "iDynTree/Wrench.h"
#include "iDynTree/SpatialMomentum.h"
#include "iDynTree/SpatialAcc.h"
#include "iDynTree/ClassicalAcc.h"
#include "iDynTree/Direction.h"
#include "iDynTree/Axis.h"

// Inertias
#include "iDynTree/RotationalInertiaRaw.h"
#include "iDynTree/SpatialInertiaRaw.h"
#include "iDynTree/SpatialInertia.h"
#include "iDynTree/ArticulatedBodyInertia.h"
#include "iDynTree/InertiaNonLinearParametrization.h"

// Transformations: Rotation and Transform
#include "iDynTree/RotationRaw.h"
#include "iDynTree/Rotation.h"
#include "iDynTree/Transform.h"
#include "iDynTree/TransformDerivative.h"
#include "iDynTree/Span.h"

// Model related data structures
#include "iDynTree/Model/Indices.h"
#include "iDynTree/Model/LinkState.h"
#include "iDynTree/Model/Link.h"
#include "iDynTree/Model/IJoint.h"
#include "iDynTree/Model/FixedJoint.h"
#include "iDynTree/Model/MovableJointImpl.h"
#include "iDynTree/Model/RevoluteJoint.h"
#include "iDynTree/Model/PrismaticJoint.h"
#include "iDynTree/Model/Traversal.h"
#include "iDynTree/Model/SolidShapes.h"
#include "iDynTree/Model/Model.h"
#include "iDynTree/Model/JointState.h"
#include "iDynTree/Model/FreeFloatingMatrices.h"
#include "iDynTree/Model/FreeFloatingState.h"
#include "iDynTree/Model/ContactWrench.h"
#include "iDynTree/Model/ModelTestUtils.h"
#include "iDynTree/Model/ModelTransformers.h"
#include "iDynTree/Model/SubModel.h"

// Kinematics & Dynamics related functions
#include "iDynTree/Model/ForwardKinematics.h"
#include "iDynTree/Model/Dynamics.h"
#include "iDynTree/Model/DenavitHartenberg.h"

// Sensors related data structures
#include "iDynTree/Sensors/Sensors.h"
#include "iDynTree/Sensors/SixAxisForceTorqueSensor.h"
#include "iDynTree/Sensors/AccelerometerSensor.h"
#include "iDynTree/Sensors/GyroscopeSensor.h"
#include "iDynTree/Sensors/ThreeAxisAngularAccelerometerSensor.h"
#include "iDynTree/Sensors/ThreeAxisForceTorqueContactSensor.h"
#include "iDynTree/Sensors/PredictSensorsMeasurements.h"

// Model loading from external formats
#include "iDynTree/ModelIO/URDFDofsImport.h"
#include "iDynTree/ModelIO/ModelLoader.h"
#include "iDynTree/ModelIO/ModelExporter.h"
#include "iDynTree/ModelIO/ModelCalibrationHelper.h"


// Estimation related classes
#include "iDynTree/Estimation/ExternalWrenchesEstimation.h"
#include "iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h"
#include "iDynTree/Estimation/SimpleLeggedOdometry.h"
#include "iDynTree/Estimation/BerdyHelper.h"
#include "iDynTree/Estimation/BerdySparseMAPSolver.h"
#include "iDynTree/Estimation/AttitudeEstimator.h"
#include "iDynTree/Estimation/AttitudeMahonyFilter.h"
#include "iDynTree/Estimation/ExtendedKalmanFilter.h"
#include "iDynTree/Estimation/AttitudeQuaternionEKF.h"

// SolidShapes related classes
#include "iDynTree/InertialParametersSolidShapesHelpers.h"

// High level interfaces
#include "iDynTree/KinDynComputations.h"

// Visualization
#include "iDynTree/Visualizer.h"

// Inverse Kinematics
#include "iDynTree/ConvexHullHelpers.h"
#include "iDynTree/InverseKinematics.h"

%}

//Wrap std::vector<BerdySensors>
namespace std {
    %template(BerdySensors) vector<iDynTree::BerdySensor>;
    %template(BerdyDynamicVariables) vector<iDynTree::BerdyDynamicVariable>;
}

//Utils
%include "iDynTree/Utils.h"

/* Note : always include headers following the inheritance order */
// Basic math classes
%include "iDynTree/MatrixDynSize.h"
%include "iDynTree/MatrixFixSize.h"
%include "iDynTree/SparseMatrix.h"
%template(SparseMatrixRowMajor) iDynTree::SparseMatrix<iDynTree::RowMajor>;
%template(SparseMatrixColMajor) iDynTree::SparseMatrix<iDynTree::ColumnMajor>;


%include "iDynTree/VectorDynSize.h"
%include "iDynTree/VectorFixSize.h"

#ifdef SWIGMATLAB
%include "./matlab/matlab_matvec.i"
#endif

#ifdef SWIGOCTAVE
%include "./octave/octave_matvec.i"
#endif

%template(Matrix1x6) iDynTree::MatrixFixSize<1,6>;
%template(Matrix2x3) iDynTree::MatrixFixSize<2,3>;
%template(Matrix3x3) iDynTree::MatrixFixSize<3,3>;
%template(Matrix4x4) iDynTree::MatrixFixSize<4,4>;
%template(Matrix6x6) iDynTree::MatrixFixSize<6,6>;
%template(Matrix6x10) iDynTree::MatrixFixSize<6,10>;
%template(Matrix10x16) iDynTree::MatrixFixSize<10,16>;

%template(Vector3) iDynTree::VectorFixSize<3>;
%template(Vector4) iDynTree::VectorFixSize<4>;
%template(Vector6) iDynTree::VectorFixSize<6>;
%template(Vector10) iDynTree::VectorFixSize<10>;
%template(Vector16) iDynTree::VectorFixSize<16>;

// Basic Vectors: Point Vectors and Spatial Vectors
%include "iDynTree/PositionRaw.h"
%include "iDynTree/Position.h"

%include "iDynTree/GeomVector3.h"

%include "iDynTree/SpatialVector.h"

%template() iDynTree::DualSpace<iDynTree::SpatialMotionVector>;
%template() iDynTree::DualSpace<iDynTree::SpatialForceVector>;

#ifdef SWIGMATLAB
%include "./matlab/matlab_spatialvec.i"
#endif

#ifdef SWIGOCTAVE
%include "./octave/octave_spatialvec.i"
#endif

%template(SpatialMotionVectorBase) iDynTree::SpatialVector<iDynTree::SpatialMotionVector>;
%template(SpatialForceVectorBase) iDynTree::SpatialVector<iDynTree::SpatialForceVector>;

%include "iDynTree/SpatialMotionVector.h"
%include "iDynTree/SpatialForceVector.h"
%include "iDynTree/Twist.h"
%include "iDynTree/Wrench.h"
%include "iDynTree/SpatialMomentum.h"
%include "iDynTree/SpatialAcc.h"
%include "iDynTree/ClassicalAcc.h"
%include "iDynTree/Direction.h"
%include "iDynTree/Axis.h"

// Inertias
%include "iDynTree/RotationalInertiaRaw.h"
%include "iDynTree/SpatialInertiaRaw.h"
%include "iDynTree/SpatialInertia.h"
%include "iDynTree/ArticulatedBodyInertia.h"
%include "iDynTree/InertiaNonLinearParametrization.h"

// Transformations: Rotation and Transform
%include "iDynTree/RotationRaw.h"
%include "iDynTree/Rotation.h"
%include "iDynTree/Transform.h"
%include "iDynTree/TransformDerivative.h"
%include "iDynTree/Span.h"
%include "iDynTree/MatrixView.h"

%template(DynamicSpan) iDynTree::Span<double, iDynTree::dynamic_extent>;
%template(DynamicMatrixView) iDynTree::MatrixView<double>;

// Model related data structures
%include "iDynTree/Model/Indices.h"
%include "iDynTree/Model/LinkState.h"
%include "iDynTree/Model/Link.h"
%include "iDynTree/Model/IJoint.h"
%include "iDynTree/Model/FixedJoint.h"
%include "iDynTree/Model/MovableJointImpl.h"

%template(MovableJointImpl1) iDynTree::MovableJointImpl<1,1>;
%template(MovableJointImpl2) iDynTree::MovableJointImpl<2,2>;
%template(MovableJointImpl3) iDynTree::MovableJointImpl<3,3>;
%template(MovableJointImpl4) iDynTree::MovableJointImpl<4,4>;
%template(MovableJointImpl5) iDynTree::MovableJointImpl<5,5>;
%template(MovableJointImpl6) iDynTree::MovableJointImpl<6,6>;

%include "iDynTree/Model/RevoluteJoint.h"
%include "iDynTree/Model/PrismaticJoint.h"
%include "iDynTree/Model/Traversal.h"
%include "iDynTree/Model/SolidShapes.h"
%include "iDynTree/Model/Model.h"
%include "iDynTree/Model/JointState.h"
%include "iDynTree/Model/FreeFloatingMatrices.h"
%include "iDynTree/Model/FreeFloatingState.h"
%include "iDynTree/Model/ContactWrench.h"
%include "iDynTree/Model/ModelTestUtils.h"
%include "iDynTree/Model/ModelTransformers.h"
%include "iDynTree/Model/SubModel.h"

%include "joints.i"

%template(SolidShapesVector) std::vector<iDynTree::SolidShape*>;
%template(LinksSolidShapesVector) std::vector< std::vector<iDynTree::SolidShape *>>;


// Kinematics & Dynamics related functions
%include "iDynTree/Model/ForwardKinematics.h"
%include "iDynTree/Model/Dynamics.h"
%include "iDynTree/Model/DenavitHartenberg.h"

// Sensors related data structures
%include "iDynTree/Sensors/Sensors.h"
%include "iDynTree/Sensors/SixAxisForceTorqueSensor.h"
%include "iDynTree/Sensors/AccelerometerSensor.h"
%include "iDynTree/Sensors/GyroscopeSensor.h"
%include "iDynTree/Sensors/ThreeAxisAngularAccelerometerSensor.h"
%include "iDynTree/Sensors/ThreeAxisForceTorqueContactSensor.h"
%include "iDynTree/Sensors/PredictSensorsMeasurements.h"

%include "sensors.i"

// Model loading from external formats
%include "iDynTree/ModelIO/URDFDofsImport.h"
%include "iDynTree/ModelIO/ModelLoader.h"
%include "iDynTree/ModelIO/ModelExporter.h"
%include "iDynTree/ModelIO/ModelCalibrationHelper.h"

// Estimation related classes
%include "iDynTree/Estimation/ExternalWrenchesEstimation.h"
%include "iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h"
%include "iDynTree/Estimation/SimpleLeggedOdometry.h"
%include "iDynTree/Estimation/BerdyHelper.h"
%include "iDynTree/Estimation/BerdySparseMAPSolver.h"
%include "iDynTree/Estimation/AttitudeEstimator.h"
%include "iDynTree/Estimation/AttitudeMahonyFilter.h"
%include "iDynTree/Estimation/ExtendedKalmanFilter.h"
%include "iDynTree/Estimation/AttitudeQuaternionEKF.h"

// SolidShapes related classes
%include "iDynTree/InertialParametersSolidShapesHelpers.h"

// High level interfaces
#ifdef SWIGPYTHON
%shared_ptr(iDynTree::KinDynComputations)
#endif
%include "iDynTree/KinDynComputations.h"

#ifdef SWIGMATLAB
%include "./matlab/matlab_mat4x4vec.i"
#endif

%template(Matrix4x4Vector) std::vector<iDynTree::Matrix4x4>;


// Visualization
%include "iDynTree/Visualizer.h"

// Inverse Kinematics
%include "iDynTree/ConvexHullHelpers.h"
%include "iDynTree/InverseKinematics.h"

