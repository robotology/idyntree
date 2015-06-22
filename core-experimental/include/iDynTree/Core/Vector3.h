/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_VECTOR_3_H
#define IDYNTREE_VECTOR_3_H

#include "IVector.h"
#include <string>

namespace iDynTree
{
    /**
     * Class providing a simple vector of 3 elements.
     *
     * \ingroup iDynTreeCore
     */
    class Vector6: public IVector
    {
    protected:
        /**
         * Storage for the VectorDynSize
         *
         * Array of 3 doubles.
         */
        double m_data[3];

    public:
        /**
         * Default constructor: initialize the elements of the array to zero.
         */
        Vector6();

        /**
         * Constructor from a C-style array.
         *
         * Print an error an build a vector full of zeros if in_size is not 3.
         */
        Vector6(const double * in_data, const unsigned int in_size);

        /**
         * Denstructor
         *
         */
        virtual ~Vector6();

        /**
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to Vector6.
         */
        ///@{
        double operator()(const unsigned int index) const;

        double& operator()(const unsigned int index);

        double getVal(const unsigned int index) const;

        bool setVal(const unsigned int index, const double new_el);

        unsigned int size() const;

        ///@}

        /**
         * Raw data accessor
         *
         * @return a const pointer to a vector of size() doubles
         */
        const double * data() const;

        /**
         * Raw data accessor
         *
         * @return a pointer to a vector of size() doubles
         */
        double * data();

        /**
         * Assign all element of the vector to 0.
         */
        void zero();

        /**
         *  @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };
}

#endif /* IDYNTREE_VECTOR_3_H */