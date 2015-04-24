
/* File : iDynTree.i */
%module iDynTree
%include "std_string.i"

%{
#include "iDynTree/Base/Position.h"
%}

/* Let's just grab the original header file here */
%include "iDynTree/Base/Position.h"