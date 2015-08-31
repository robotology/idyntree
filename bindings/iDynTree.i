
/* File : iDynTree.i */
%module iDynTree

%include "std_string.i"

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

// Basic math classes
#include "iDynTree/Core/IMatrix.h"
#include "iDynTree/Core/IVector.h"
#include "iDynTree/Core/MatrixDynSize.h"
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

// Transformations: Rotation and Transform
#include "iDynTree/Core/RotationRaw.h"
#include "iDynTree/Core/RotationSemantics.h"
#include "iDynTree/Core/Rotation.h"
#include "iDynTree/Core/TransformSemantics.h"
#include "iDynTree/Core/Transform.h"

// Model related data structures
#include "iDynTree/Model/Indeces.h"
#include "iDynTree/Model/LinkState.h"
#include "iDynTree/Model/IJointStateInterfaces.h"
#include "iDynTree/Model/Link.h"
#include "iDynTree/Model/IJoint.h"
#include "iDynTree/Model/FixedJoint.h"
#include "iDynTree/Model/MovableJointImpl.h"
#include "iDynTree/Model/RevoluteJoint.h"
#include "iDynTree/Model/Traversal.h"
#include "iDynTree/Model/Model.h"

// Model loading from external formats
#include "iDynTree/ModelIO/URDFModelImport.h"

// Sensors related data structures
#include "iDynTree/Sensors/Sensors.hpp"
#include "iDynTree/Sensors/SixAxisFTSensor.hpp"

// Sensors loading from external formats
#include "iDynTree/ModelIO/URDFSensorsImport.h"

// Regressors related data structures
#include "iDynTree/Regressors/DynamicsRegressorParameters.h"
#include "iDynTree/Regressors/DynamicsRegressorGenerator.h"

// High level interfaces
#include "iDynTree/HighLevel/DynamicsComputations.h"

%}

/**
 * macro for 3D, Spatial vectors classes and traits classes instanciations
 */

%define TEMPLATE_WRAP_MOTION_FORCE(class, wrapMotionForce, type1, postfix)
#define add 1
#define addTemplateType1 add ## type1
#if addTemplateType1 != add
// type1 is not empty, we use it as the first template instanciation parameter
#define templateType1 iDynTree:: ## type1 ## ,
#else
#define templateType1
#endif

#define wrapMotionForce
// SWIG does not support "if defined", neather boolean operators within "ifdef"
#ifdef WRAP_MOTION_FORCE
#define WRAP_MOTION
#define WRAP_FORCE
#endif

#ifdef WRAP_MOTION
%template(class ## _ ## type1 ## _LinearMotionVector3 ## postfix) iDynTree:: ## class ## <templateType1 iDynTree::LinearMotionVector3 ## postfix>;
%template(class ## _ ## type1 ## _AngularMotionVector3 ## postfix) iDynTree:: ## class ## <templateType1 iDynTree::AngularMotionVector3 ## postfix>;
#endif
#ifdef WRAP_FORCE
%template(class ## _ ## type1 ## _LinearForceVector3 ## postfix) iDynTree:: ## class ## <templateType1 iDynTree::LinearForceVector3 ## postfix>;
%template(class ## _ ## type1 ## _AngularForceVector3 ## postfix) iDynTree:: ## class ## <templateType1 iDynTree::AngularForceVector3 ## postfix>;
#endif

#undef add
#undef addTemplateType1
#undef templateType1
#undef WRAP_MOTION
#undef WRAP_FORCE
#undef WRAP_MOTION_FORCE
%enddef


/* Note : always include headers following the inheritance order */
// Basic math classes
%include "iDynTree/Core/IMatrix.h"
%include "iDynTree/Core/IVector.h"
%include "iDynTree/Core/MatrixDynSize.h"
%include "iDynTree/Core/MatrixFixSize.h"


%include "iDynTree/Core/VectorDynSize.h"
%include "iDynTree/Core/VectorFixSize.h"

#ifdef SWIGMATLAB
%include "./matlab/matlab_matvec.i"
#endif

%template(Matrix3x3) iDynTree::MatrixFixSize<3,3>;
%template(Matrix4x4) iDynTree::MatrixFixSize<4,4>;
%template(Matrix6x6) iDynTree::MatrixFixSize<6,6>;
%template(Matrix6x10) iDynTree::MatrixFixSize<6,10>;

%template(Vector3) iDynTree::VectorFixSize<3>;
%template(Vector6) iDynTree::VectorFixSize<6>;
%template(Vector10) iDynTree::VectorFixSize<10>;

// Basic Vectors: Point Vectors and Spatial Vectors
%include "iDynTree/Core/PositionRaw.h"
%include "iDynTree/Core/PositionSemantics.h"
%include "iDynTree/Core/Position.h"

%include "iDynTree/Core/PrivateMotionForceVertorAssociations.h"

TEMPLATE_WRAP_MOTION_FORCE(MotionForce_traits, WRAP_MOTION_FORCE,,)

TEMPLATE_WRAP_MOTION_FORCE(MotionDerivativeOf, WRAP_MOTION_FORCE, AngularMotionVector3,)
TEMPLATE_WRAP_MOTION_FORCE(MotionDerivativeOf, WRAP_MOTION_FORCE, LinearMotionVector3,)

TEMPLATE_WRAP_MOTION_FORCE(ConvertSem2motionForceTraits, WRAP_MOTION_FORCE,,Semantics)

TEMPLATE_WRAP_MOTION_FORCE(DualMotionForceSemanticsT, WRAP_MOTION_FORCE,,Semantics)

%template(SpatialMotionForceVectorT_traits__SpatialMotionVector) iDynTree::SpatialMotionForceVectorT_traits<iDynTree::SpatialMotionVector>;
%template(SpatialMotionForceVectorT_traits__SpatialForceVector) iDynTree::SpatialMotionForceVectorT_traits<iDynTree::SpatialForceVector>;


%include "iDynTree/Core/GeomVector3.h"

TEMPLATE_WRAP_MOTION_FORCE(GeomVector3Semantics, WRAP_MOTION_FORCE,,Semantics)

TEMPLATE_WRAP_MOTION_FORCE(GeomVector3, WRAP_MOTION_FORCE,,)

%include "iDynTree/Core/MotionVector3.h"
%include "iDynTree/Core/ForceVector3.h"

TEMPLATE_WRAP_MOTION_FORCE(ForceVector3Semantics, WRAP_FORCE,,Semantics)

TEMPLATE_WRAP_MOTION_FORCE(MotionVector3, WRAP_MOTION,,)

TEMPLATE_WRAP_MOTION_FORCE(ForceVector3, WRAP_FORCE,,)

%include "iDynTree/Core/LinearMotionVector3.h"
%include "iDynTree/Core/AngularMotionVector3.h"
%include "iDynTree/Core/LinearForceVector3.h"
%include "iDynTree/Core/AngularForceVector3.h"

%include "iDynTree/Core/SpatialVector.h"

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

// Transformations: Rotation and Transform
%include "iDynTree/Core/RotationRaw.h"
%include "iDynTree/Core/RotationSemantics.h"
%include "iDynTree/Core/Rotation.h"
%include "iDynTree/Core/TransformSemantics.h"
%include "iDynTree/Core/Transform.h"

// Model related data structures
%include "iDynTree/Model/Indeces.h"
%include "iDynTree/Model/LinkState.h"
%include "iDynTree/Model/IJointStateInterfaces.h"
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

// Model loading from external formats
%include "iDynTree/ModelIO/URDFModelImport.h"

// Sensors related data structures
%include "iDynTree/Sensors/Sensors.hpp"
%include "iDynTree/Sensors/SixAxisFTSensor.hpp"

%include "sensors.i"

// Sensors loading from external formats
%include "iDynTree/ModelIO/URDFSensorsImport.h"

// Regressors related data structures
%include "iDynTree/Regressors/DynamicsRegressorParameters.h"
%include "iDynTree/Regressors/DynamicsRegressorGenerator.h"

// High level interfaces
%include "iDynTree/HighLevel/DynamicsComputations.h"

