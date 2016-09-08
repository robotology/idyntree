/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_UTILS_H
#define IDYNTREE_UTILS_H

#include <cstddef>

namespace iDynTree
{

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

    /**
     * Helper function for reporting warnings in iDynTree
     *
     */
    void reportWarning(const char * className, const char* methodName, const char * errorMessage);

    /**
     * Convert a double from degrees to radians.
     */
    double deg2rad(const double valueInDeg);

    /**
     * Convert a double from radians to degree.
     */
    double rad2deg(const double valueInRad);

    /**
     * Simple structure describing a range of rows or columns in a vector or a matrix.
     * The offset attributes indicates the index of the first element of the range, while
     * the size indicates the size of the range.
     *
     * Example: offset = 2, size = 3 measn the elements 2 3 4 .
     */
    struct IndexRange
    {
        std::ptrdiff_t offset;
        std::ptrdiff_t size;

        bool isValid() const;
        static IndexRange InvalidRange();
    };


}

#endif
