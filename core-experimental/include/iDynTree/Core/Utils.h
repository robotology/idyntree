/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

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
    bool reportErrorIf(bool condition, const char * className, const char* methodName, const char * errorMessage);
}