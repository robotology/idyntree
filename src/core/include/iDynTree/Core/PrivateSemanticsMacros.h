/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_PRIVATE_SEMANTICS_MACROS_H
#define IDYNTREE_PRIVATE_SEMANTICS_MACROS_H

/**
 * macro for activating the semantic checking.
 *
 * This macro defines the assert function embedding the semantic checks for
 * geometrical relations.
 *
 * It also define a iDynTreeSemanticsOp macro for operation that should
 * be converted to NOP when compiling iDynTree without semantics support
 * (for example Semantics objects initialization).
 *
 */
#if defined NDEBUG || !defined IDYNTREE_USES_SEMANTICS
  #define iDynTreeAssert(semCheckBoolOut) ((void)0)
  #define iDynTreeSemanticsOp(op) ((void)0)
  #define IDYNTREE_DONT_USE_SEMANTICS
#else
  #ifdef IDYNTREE_COMPILE_BINDINGS
    #define iDynTreeAssert(semCheckBoolOut) ((void) ((semCheckBoolOut) ? (void)0 : assertWoAbort(#semCheckBoolOut, __FILE__, __PRETTY_FUNCTION__, __LINE__)))
    #define iDynTreeSemanticsOp(op) op
  #else
    #define iDynTreeAssert(semCheckBoolOut) assert(semCheckBoolOut)
    #define iDynTreeSemanticsOp(op) op
  #endif
#endif

#endif /* IDYNTREE_PRIVATE_SEMANTICS_MACROS_H */
