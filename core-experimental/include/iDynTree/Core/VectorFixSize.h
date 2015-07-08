/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_VECTOR_FIX_SIZE_H
#define IDYNTREE_VECTOR_FIX_SIZE_H

#include <iDynTree/Core/IVector.h>
#include <iDynTree/Core/Utils.h>
#include <string>
#include <sstream>
#include <cstring>

namespace iDynTree
{
    /**
     * Class providing a simple vector of N elements.
     *  The size of the vector is known at compile time,
     *  and it enables to avoid using dynamic memory allocation.
     *
     * \ingroup iDynTreeCore
     */
    template<unsigned int VecSize> class VectorFixSize: public IVector
    {
    protected:
        /**
         * Storage for the VectorDynSize
         *
         * Array of 6 doubles.
         */
        double m_data[VecSize];

    public:
        /**
         * Default constructor: initialize the elements of the array to zero.
         */
        VectorFixSize();

        /**
         * Constructor from a C-style array.
         *
         * Print an error an build a vector full of zeros if in_size is not size().
         */
        VectorFixSize(const double * in_data, const unsigned int in_size);

        /**
         * Denstructor
         *
         */
        virtual ~VectorFixSize();

        /**
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to VectorFixSize.
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
        virtual std::string toString() const;

        virtual std::string reservedToString() const;
        ///@}

    };

    //Implementation
    template<unsigned int VecSize>
    VectorFixSize<VecSize>::VectorFixSize()
    {
        this->zero();
    }


    template<unsigned int VecSize>
    VectorFixSize<VecSize>::VectorFixSize(const double* in_data,
                                 const unsigned int in_size)
    {
        if( in_size != VecSize )
        {
            reportError("VectorFixSize","constructor","input vector does not have the right number of elements");
            this->zero();
        }
        else
        {
            memcpy(this->m_data,in_data,sizeof(double)*VecSize);
        }
    }

    template<unsigned int VecSize>
    VectorFixSize<VecSize>::~VectorFixSize()
    {

    }

    template<unsigned int VecSize>
    void VectorFixSize<VecSize>::zero()
    {
        for(unsigned int i=0; i < this->size(); i++ )
        {
            this->m_data[i] = 0.0;
        }
    }


    template<unsigned int VecSize>
    double* VectorFixSize<VecSize>::data()
    {
        return this->m_data;
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::data() const
    {
        return this->m_data;
    }

    template<unsigned int VecSize>
    unsigned int VectorFixSize<VecSize>::size() const
    {
        return VecSize;
    }

    template<unsigned int VecSize>
    double VectorFixSize<VecSize>::operator()(const unsigned int index) const
    {
        return this->m_data[index];
    }

    template<unsigned int VecSize>
    double & VectorFixSize<VecSize>::operator()(const unsigned int index)
    {
        return this->m_data[index];
    }


    template<unsigned int VecSize>
    double VectorFixSize<VecSize>::getVal(const unsigned int index) const
    {
        if( index >= this->size() )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return 0.0;
        }

        return this->m_data[index];
    }

    template<unsigned int VecSize>
    bool VectorFixSize<VecSize>::setVal(const unsigned int index, const double new_el)
    {
        if( index >= this->size() )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return false;
        }

        this->m_data[index] = new_el;

        return true;
    }

    template<unsigned int VecSize>
    std::string VectorFixSize<VecSize>::toString() const
    {
        std::stringstream ss;

        for(unsigned int i=0; i < this->size(); i++ )
        {
            ss << this->m_data[i] << " ";
        }

        return ss.str();
    }

    template<unsigned int VecSize>
    std::string VectorFixSize<VecSize>::reservedToString() const
    {
        std::stringstream ss;

        for(unsigned int i=0; i < this->size(); i++ )
        {
            ss << this->m_data[i] << " ";
        }

        return ss.str();
    }

    // Explicit instantiations
    // The explicit instantiations are the only ones that can be used in the API
    //  and the only ones that users are supposed to manipulate manipulate
    // Add all the explicit instantiation that can be useful, but remember to add
    // them also in the iDynTree.i SWIG file
    typedef VectorFixSize<3> Vector3;
    typedef VectorFixSize<6> Vector6;
    typedef VectorFixSize<10> Vector10;

}

#endif /* IDYNTREE_VECTOR_FIX_SIZE_H */