
/* File : iDynTree.i */
%module iDynTree
%include "std_string.i"

%{
#include "iDynTree/Core/PositionRaw.h"
#include "iDynTree/Core/PositionSemantics.h"
#include "iDynTree/Core/Position.h"
%}

/* Let's just grab the original header file here */
%include "iDynTree/Core/PositionRaw.h"
%include "iDynTree/Core/PositionSemantics.h"
%include "iDynTree/Core/Position.h"