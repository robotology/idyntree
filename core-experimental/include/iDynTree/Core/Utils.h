/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_UTILS_H
#define IDYNTREE_UTILS_H

/**
 * macro for activating the semantic checking.
 *
 * This macro defines the assert function embedding the semantic checks for
 * geometrical relations.
 *
 */
#if defined NDEBUG || !defined IDYNTREE_USES_SEMANTICS
  #define iDynTreeAssert(semCheckBoolOut) ((void)0)
#else
  #ifdef IDYNTREE_COMPILE_BINDINGS
    #define iDynTreeAssert(semCheckBoolOut) ((void) ((semCheckBoolOut) ? (void)0 : assertWoAbort(#semCheckBoolOut, __FILE__, __PRETTY_FUNCTION__, __LINE__)))
  #else
    #define iDynTreeAssert(semCheckBoolOut) assert(semCheckBoolOut)
  #endif
#endif


namespace iDynTree
{
    /**
     * \defgroup iDynTreeCore iDynTree core data structures
     *
     * iDynTree provides basic data structures to implement kinemanics and dynamics
     * algorithms.
     *
     *
     */


    extern int UNKNOWN;

    /// Default tolerance for methods with a tolerance, setted to 1e-10
    extern double DEFAULT_TOL;

    /**
     * Function embedding the semantic checks
     *
     * This function can throw an exception if the semantic check detects an error (returns False).
     */
    void assertWoAbort(const char * semCheck, const char * file, const char* func, int line);

    /**
     * Helper class for semantic checking.
     *
     * Return true if two values are equal, or if one of the two is unknown
     * All negative values are used for represent an unknown value.
     */
    bool checkEqualOrUnknown(const int op1, const int op2);

    /**
     * Helper function for reporting error if the semantic check fails.
     *
     */
    void reportError(const char * className, const char* methodName, const char * errorMessage);

    /**
     * Call report error if condition is true
     */
    bool reportErrorIf(bool condition, const char * className_methodName, const char * errorMessage);
}

#endif