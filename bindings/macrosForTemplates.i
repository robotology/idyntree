#ifndef iDynTree_macrosForTemplates_i
#define iDynTree_macrosForTemplates_i

/**
 * macro for 3D, Spatial vectors classes and traits classes instanciations
 */

%define TEMPLATE_WRAP_MOTION_FORCE(class, wrapMotionForce, setTemplateName, type1, postfix)
#undef add
#define add ## type1
#ifndef add
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

#define setTemplateName
#ifdef SET_NAME_FOR_WRAPPER
#define templateNameLM class ## _ ## type1 ## _LinearMotionVector3 ## postfix
#define templateNameAM class ## _ ## type1 ## _AngularMotionVector3 ## postfix
#define templateNameLF class ## _ ## type1 ## _LinearForceVector3 ## postfix
#define templateNameAF class ## _ ## type1 ## _AngularForceVector3 ## postfix
#else
#ifdef NO_NAME_FOR_WRAPPER
#define templateNameLM
#define templateNameAM
#define templateNameLF
#define templateNameAF
#else
#warning wrong_parameter_NAME_FOR_WRAPPER
#endif
#endif

#ifdef WRAP_MOTION
%template(templateNameLM) iDynTree:: ## class ## <templateType1 iDynTree::LinearMotionVector3 ## postfix>;
%template(templateNameAM) iDynTree:: ## class ## <templateType1 iDynTree::AngularMotionVector3 ## postfix>;
#endif
#ifdef WRAP_FORCE
%template(templateNameLF) iDynTree:: ## class ## <templateType1 iDynTree::LinearForceVector3 ## postfix>;
%template(templateNameAF) iDynTree:: ## class ## <templateType1 iDynTree::AngularForceVector3 ## postfix>;
#endif
#ifndef WRAP_MOTION
#ifndef WRAP_FORCE
#warning wrong_parameter_WRAP_MOTIONFORCE
#endif
#endif

#undef add ## type1
#undef templateType1
#undef setTemplateName
#undef templateNameLM
#undef templateNameAM
#undef templateNameLF
#undef templateNameAF
#undef WRAP_MOTION
#undef WRAP_FORCE
#undef WRAP_MOTION_FORCE
%enddef


#endif
