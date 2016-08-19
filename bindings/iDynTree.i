
/* File : iDynTree.i */
%module iDynTree

%include "std_string.i"

// Wrap the std::vector<std::string> params
%include "std_vector.i"

namespace std {
    %template(StringVector) vector<string>;
}

// Ignore some methods to avoid warnings
%include "./ignore.i"

// macros for class templates handling
%include "./macrosForTemplates.i"

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

//Utils
#include "iDynTree/Core/Utils.h"

// Basic math classes
#include "iDynTree/Core/MatrixDynSize.h"
#include "iDynTree/Core/MatrixFixSize.h"

#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/VectorFixSize.h"

// Basic Vectors: Point Vectors and Spatial Vectors
#include "iDynTree/Core/PositionRaw.h"
#include "iDynTree/Core/PositionSemantics.h"
#include "iDynTree/Core/Position.h"
#include "iDynTree/Core/LinearMotionVector3.h"
#include "iDynTree/Core/LinearForceVector3.h"
#include "iDynTree/Core/AngularMotionVector3.h"
#include "iDynTree/Core/AngularForceVector3.h"
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

// Transformations: Rotation and Transform
#include "iDynTree/Core/RotationRaw.h"
#include "iDynTree/Core/RotationSemantics.h"
#include "iDynTree/Core/Rotation.h"
#include "iDynTree/Core/TransformSemantics.h"
#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/TransformDerivative.h"

// Model related data structures
#include "iDynTree/Model/Indeces.h"
#include "iDynTree/Model/LinkState.h"
#include "iDynTree/Model/Link.h"
#include "iDynTree/Model/IJoint.h"
#include "iDynTree/Model/FixedJoint.h"
#include "iDynTree/Model/MovableJointImpl.h"
#include "iDynTree/Model/RevoluteJoint.h"
#include "iDynTree/Model/Traversal.h"
#include "iDynTree/Model/Model.h"
#include "iDynTree/Model/JointState.h"
#include "iDynTree/Model/FreeFloatingMassMatrix.h"
#include "iDynTree/Model/FreeFloatingState.h"
#include "iDynTree/Model/ContactWrench.h"

// Kinematics & Dynamics related functions
#include "iDynTree/Model/ForwardKinematics.h"
#include "iDynTree/Model/Dynamics.h"

// Sensors related data structures
#include "iDynTree/Sensors/Sensors.h"
#include "iDynTree/Sensors/SixAxisFTSensor.h"
#include "iDynTree/Sensors/AccelerometerSensor.h"
#include "iDynTree/Sensors/GyroscopeSensor.h"
#include "iDynTree/Sensors/PredictSensorsMeasurements.h"

// Model loading from external formats
#include "iDynTree/ModelIO/URDFModelImport.h"
#include "iDynTree/ModelIO/URDFGenericSensorsImport.h"
#include "iDynTree/ModelIO/ModelLoader.h"

// Estimation related classes
#include "iDynTree/Estimation/ExternalWrenchesEstimation.h"
#include "iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h"
#include "iDynTree/Estimation/SimpleLeggedOdometry.h"
#include "iDynTree/Estimation/BerdyHelper.h"

// Regressors related data structures
#include "iDynTree/Regressors/DynamicsRegressorParameters.h"
#include "iDynTree/Regressors/DynamicsRegressorGenerator.h"

// High level interfaces
#include "iDynTree/KinDynComputations.h"

// Legacy high level interfaces
#include "iDynTree/HighLevel/DynamicsComputations.h"

%}

//Wrap std::vector<BerdySensors>
namespace std {
    %template(BerdySensors) vector<iDynTree::BerdySensor>;
}

//Utils
%include "iDynTree/Core/Utils.h"

/* Note : always include headers following the inheritance order */
// Basic math classes
%include "iDynTree/Core/MatrixDynSize.h"
%include "iDynTree/Core/MatrixFixSize.h"


%include "iDynTree/Core/VectorDynSize.h"
%include "iDynTree/Core/VectorFixSize.h"

#ifdef SWIGMATLAB
%include "./matlab/matlab_matvec.i"
#endif

#ifdef SWIGOCTAVE
%include "./octave/octave_matvec.i"
#endif

%template(Matrix3x3) iDynTree::MatrixFixSize<3,3>;
%template(Matrix4x4) iDynTree::MatrixFixSize<4,4>;
%template(Matrix6x6) iDynTree::MatrixFixSize<6,6>;
%template(Matrix6x10) iDynTree::MatrixFixSize<6,10>;
%template(Matrix10x16) iDynTree::MatrixFixSize<10,16>;

%template(Vector3) iDynTree::VectorFixSize<3>;
%template(Vector6) iDynTree::VectorFixSize<6>;
%template(Vector10) iDynTree::VectorFixSize<10>;
%template(Vector16) iDynTree::VectorFixSize<16>;

// Basic Vectors: Point Vectors and Spatial Vectors
%include "iDynTree/Core/PositionRaw.h"
%include "iDynTree/Core/PositionSemantics.h"
%include "iDynTree/Core/Position.h"

%include "iDynTree/Core/PrivateMotionForceVertorAssociations.h"

TEMPLATE_WRAP_MOTION_FORCE(MotionForce_traits, WRAP_MOTION_FORCE, NO_NAME_FOR_WRAPPER,,)

TEMPLATE_WRAP_MOTION_FORCE(MotionDerivativeOf, WRAP_MOTION_FORCE, NO_NAME_FOR_WRAPPER, AngularMotionVector3,)
TEMPLATE_WRAP_MOTION_FORCE(MotionDerivativeOf, WRAP_MOTION_FORCE, NO_NAME_FOR_WRAPPER, LinearMotionVector3,)

TEMPLATE_WRAP_MOTION_FORCE(ConvertSem2motionForceTraits, WRAP_MOTION_FORCE, NO_NAME_FOR_WRAPPER,,Semantics)

TEMPLATE_WRAP_MOTION_FORCE(DualMotionForceSemanticsT, WRAP_MOTION_FORCE, NO_NAME_FOR_WRAPPER,,Semantics)

%template() iDynTree::SpatialMotionForceVectorT_traits<iDynTree::SpatialMotionVector>;
%template() iDynTree::SpatialMotionForceVectorT_traits<iDynTree::SpatialForceVector>;


%include "iDynTree/Core/GeomVector3.h"

TEMPLATE_WRAP_MOTION_FORCE(GeomVector3Semantics, WRAP_MOTION_FORCE, SET_NAME_FOR_WRAPPER,,Semantics)

TEMPLATE_WRAP_MOTION_FORCE(GeomVector3, WRAP_MOTION_FORCE, SET_NAME_FOR_WRAPPER,,)

%include "iDynTree/Core/MotionVector3.h"
%include "iDynTree/Core/ForceVector3.h"

TEMPLATE_WRAP_MOTION_FORCE(ForceVector3Semantics, WRAP_FORCE, SET_NAME_FOR_WRAPPER,,Semantics)

TEMPLATE_WRAP_MOTION_FORCE(MotionVector3, WRAP_MOTION, SET_NAME_FOR_WRAPPER,,)

TEMPLATE_WRAP_MOTION_FORCE(ForceVector3, WRAP_FORCE, SET_NAME_FOR_WRAPPER,,)

%include "iDynTree/Core/LinearMotionVector3.h"
%include "iDynTree/Core/AngularMotionVector3.h"
%include "iDynTree/Core/LinearForceVector3.h"
%include "iDynTree/Core/AngularForceVector3.h"

%include "iDynTree/Core/SpatialVector.h"

%template() iDynTree::DualSpace<iDynTree::SpatialMotionVector>;
%template() iDynTree::DualSpace<iDynTree::SpatialForceVector>;

#ifdef SWIGMATLAB
%include "./matlab/matlab_spatialvec.i"
#endif

#ifdef SWIGOCTAVE
%include "./octave/octave_spatialvec.i"
#endif

%template(SpatialMotionVectorSemanticsBase) iDynTree::SpatialVectorSemantics<iDynTree::LinearMotionVector3Semantics,iDynTree::AngularMotionVector3Semantics>;
%template(SpatialForceVectorSemanticsBase) iDynTree::SpatialVectorSemantics<iDynTree::LinearForceVector3Semantics,iDynTree::AngularForceVector3Semantics>;

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

// Transformations: Rotation and Transform
%include "iDynTree/Core/RotationRaw.h"
%include "iDynTree/Core/RotationSemantics.h"
%include "iDynTree/Core/Rotation.h"
%include "iDynTree/Core/TransformSemantics.h"
%include "iDynTree/Core/Transform.h"
%include "iDynTree/Core/TransformDerivative.h"


// Model related data structures
%include "iDynTree/Model/Indeces.h"
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
%include "iDynTree/Model/Traversal.h"
%include "iDynTree/Model/Model.h"
%include "iDynTree/Model/JointState.h"
%include "iDynTree/Model/FreeFloatingMassMatrix.h"
%include "iDynTree/Model/FreeFloatingState.h"
%include "iDynTree/Model/ContactWrench.h"

%include "joints.i"

// Kinematics & Dynamics related functions
%include "iDynTree/Model/ForwardKinematics.h"
%include "iDynTree/Model/Dynamics.h"

// Sensors related data structures
%include "iDynTree/Sensors/Sensors.h"
%include "iDynTree/Sensors/SixAxisFTSensor.h"
%include "iDynTree/Sensors/AccelerometerSensor.h"
%include "iDynTree/Sensors/GyroscopeSensor.h"
%include "iDynTree/Sensors/PredictSensorsMeasurements.h"

%include "sensors.i"

// Model loading from external formats
%include "iDynTree/ModelIO/URDFModelImport.h"
%include "iDynTree/ModelIO/URDFGenericSensorsImport.h"
%include "iDynTree/ModelIO/ModelLoader.h"

// Estimation related classes
%include "iDynTree/Estimation/ExternalWrenchesEstimation.h"
%include "iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h"
%include "iDynTree/Estimation/SimpleLeggedOdometry.h"
%include "iDynTree/Estimation/BerdyHelper.h"

// Regressors related data structures
%include "iDynTree/Regressors/DynamicsRegressorParameters.h"
%include "iDynTree/Regressors/DynamicsRegressorGenerator.h"

// High level interfaces
%include "iDynTree/KinDynComputations.h"

// Legacy high level interfaces
%include "iDynTree/HighLevel/DynamicsComputations.h"

