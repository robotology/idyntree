/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_VECTOR_H
#define IDYNTREE_I_VECTOR_H


namespace iDynTree
{
    /**
     * Interface (i.e. pure abstract class) exposed by
     * Vector-like classes.
     */
    class IVector
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IVector() = 0;

        /**
         * Return a vector element by value.
         *
         * No input checking.
         *
         */
        virtual double operator()(const unsigned int index) const;

        /**
         * Return a reference to an element value.
         *
         * No input checking.
         */
        virtual double& operator()(const unsigned int index);

         /**
         * Return an element of the vector by value.
         *
         * Perform boundary checking, print an error message and
         * returns 0.0 if an element outside the vector size is requested.
         *
         */
        virtual double getVal(const unsigned int index) const = 0;

        /**
         * Set an elements of the vector.
         *
         * Perform boundary checking, prints an error message and
         * returns false if it is requested to set an element outside
         * the vector boundaries.
         *
         */
        virtual bool setVal(const unsigned int index, const double new_el) = 0;


        /**
         * Return the size of the vector.
         */
        virtual unsigned int size() const = 0;


    };
}

#endif /* IDYNTREE_VECTOR_DYN_SIZE_H */