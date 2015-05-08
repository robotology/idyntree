
/* File : iDynTree.i */
%module iDynTree
%include "std_string.i"

%{
#include "iDynTree/Core/IMatrix.h"
#include "iDynTree/Core/IVector.h"
#include "iDynTree/Core/MatrixDynSize.h"
#include "iDynTree/Core/PositionRaw.h"
#include "iDynTree/Core/PositionSemantics.h"
#include "iDynTree/Core/Position.h"
#include "iDynTree/Core/RotationRaw.h"
#include "iDynTree/Core/RotationSemantics.h"
#include "iDynTree/Core/Rotation.h"
#include "iDynTree/Core/TransformRaw.h"
#include "iDynTree/Core/TransformSemantics.h"
#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/VectorDynSize.h"
%}

/* Let's just grab the original header file here */
%include "iDynTree/Core/IMatrix.h"
%include "iDynTree/Core/IVector.h"
%include "iDynTree/Core/MatrixDynSize.h"
%include "iDynTree/Core/PositionRaw.h"
%include "iDynTree/Core/PositionSemantics.h"
%include "iDynTree/Core/Position.h"
%include "iDynTree/Core/RotationRaw.h"
%include "iDynTree/Core/RotationSemantics.h"
%include "iDynTree/Core/Rotation.h"
%include "iDynTree/Core/TransformRaw.h"
%include "iDynTree/Core/TransformSemantics.h"
%include "iDynTree/Core/Transform.h"
%include "iDynTree/Core/VectorDynSize.h"