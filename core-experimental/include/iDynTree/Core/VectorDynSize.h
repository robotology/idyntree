/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_DYNAMIC_SIZE_VECTOR_H
#define IDYNTREE_DYNAMIC_SIZE_VECTOR_H

#include "IVector.h"
#include <string>

namespace iDynTree
{
    /**
     * Class providing a simple form of vector with dynamic size.
     */
    class VectorDynSize: public IVector
    {
    protected:
        /**
         * Storage for the VectorDynSize
         *
         * Pointer to an area of size() doubles, managed by this class.
         */
        double * m_data;
        unsigned int m_size;

    public:
        /**
         * Default constructor: initialize the size of the array to zero.
         */
        VectorDynSize();

        /**
         * Constructor from the size, all the element assigned to 0
         *
         * @param _size the desired size of the array.
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(unsigned int _size);

        /**
         * Constructor from a C-style array.
         *
         * Build
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(const double * in_data, const unsigned int in_size);

        /**
         * Denstructor
         *
         * \warning performs dynamic memory allocation operations
         */
        virtual ~VectorDynSize();

        /**
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to PositionRaw.
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
         * Change the size of the vector, without preserving old content.
         *
         * @param newSize the new size of the vector
         * \warning performs dynamic memory allocation operations
         */
        void resize(const unsigned int newSize);


        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        ///@}

    };
}

#endif /* IDYNTREE_VECTOR_DYN_SIZE_H */