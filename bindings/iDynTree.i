
/* File : iDynTree.i */
%module iDynTree

%include "std_string.i"
%include "std_vector.i"

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
#include "iDynTree/Core/Utils.h"

// Basic math classes
#include "iDynTree/Core/MatrixDynSize.h"
#include "iDynTree/Core/MatrixFixSize.h"
#include "iDynTree/Core/SparseMatrix.h"

#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/VectorFixSize.h"

// Basic Vectors: Point Vectors and Spatial Vectors
#include "iDynTree/Core/PositionRaw.h"
#include "iDynTree/Core/Position.h"
#include "iDynTree/Core/SpatialForceVector.h"
#include "iDynTree/Core/SpatialMotionVector.h"
#include "iDynTree/Core/Twist.h"
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Core/SpatialMomentum.h"
#include "iDynTree/Core/SpatialAcc.h"
#include "iDynTree/Core/ClassicalAcc.h"
#include "iDynTree/Core/Direction.h"
#include "iDynTree/Core/Axis.h"

// Inertias
#include "iDynTree/Core/RotationalInertiaRaw.h"
#include "iDynTree/Core/SpatialInertiaRaw.h"
#include "iDynTree/Core/SpatialInertia.h"
#include "iDynTree/Core/ArticulatedBodyInertia.h"
#include "iDynTree/Core/InertiaNonLinearParametrization.h"

// Transformations: Rotation and Transform
#include "iDynTree/Core/RotationRaw.h"
#include "iDynTree/Core/Rotation.h"
#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/TransformDerivative.h"
#include "iDynTree/Core/Span.h"

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

// Kinematics & Dynamics related functions
#include "iDynTree/Model/ForwardKinematics.h"
#include "iDynTree/Model/Dynamics.h"

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
%include "iDynTree/Core/Utils.h"

/* Note : always include headers following the inheritance order */
// Basic math classes
%include "iDynTree/Core/MatrixDynSize.h"
%include "iDynTree/Core/MatrixFixSize.h"
%include "iDynTree/Core/SparseMatrix.h"
%template(SparseMatrixRowMajor) iDynTree::SparseMatrix<iDynTree::RowMajor>;
%template(SparseMatrixColMajor) iDynTree::SparseMatrix<iDynTree::ColumnMajor>;


%include "iDynTree/Core/VectorDynSize.h"
%include "iDynTree/Core/VectorFixSize.h"

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
%include "iDynTree/Core/PositionRaw.h"
%include "iDynTree/Core/Position.h"

%include "iDynTree/Core/GeomVector3.h"

%include "iDynTree/Core/SpatialVector.h"

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

%include "iDynTree/Core/SpatialMotionVector.h"
%include "iDynTree/Core/SpatialForceVector.h"
%include "iDynTree/Core/Twist.h"
%include "iDynTree/Core/Wrench.h"
%include "iDynTree/Core/SpatialMomentum.h"
%include "iDynTree/Core/SpatialAcc.h"
%include "iDynTree/Core/ClassicalAcc.h"
%include "iDynTree/Core/Direction.h"
%include "iDynTree/Core/Axis.h"

// Inertias
%include "iDynTree/Core/RotationalInertiaRaw.h"
%include "iDynTree/Core/SpatialInertiaRaw.h"
%include "iDynTree/Core/SpatialInertia.h"
%include "iDynTree/Core/ArticulatedBodyInertia.h"
%include "iDynTree/Core/InertiaNonLinearParametrization.h"

// Transformations: Rotation and Transform
%include "iDynTree/Core/RotationRaw.h"
%include "iDynTree/Core/Rotation.h"
%include "iDynTree/Core/Transform.h"
%include "iDynTree/Core/TransformDerivative.h"
%include "iDynTree/Core/Span.h"
%include "iDynTree/Core/MatrixView.h"

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

%include "joints.i"

%template(SolidShapesVector) std::vector<iDynTree::SolidShape*>;
%template(LinksSolidShapesVector) std::vector< std::vector<iDynTree::SolidShape *>>;


// Kinematics & Dynamics related functions
%include "iDynTree/Model/ForwardKinematics.h"
%include "iDynTree/Model/Dynamics.h"

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

