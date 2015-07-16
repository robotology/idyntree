/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FORCE_VECTOR_3_H
#define IDYNTREE_FORCE_VECTOR_3_H

#include <iDynTree/Core/GeomVector3.h>

namespace iDynTree
{
    //class LinearForceVector3;
    //class AngularForceVector3;
    
    /**
     * Class providing the raw coordinates and semantics for any force vector
     *
     * \ingroup iDynTreeCore
     *
     * A force vector can be used to describe a linear or angular momentum or force.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to every force vectors.
     *
     */
    template <class ForceT, template <class MotionForceT> class ForceConversionsT>
    class ForceVector3: public GeomVector3<ForceT, ForceConversionsT>
    {
    public:
        /**
         * constructors
         */
        ForceVector3();
        ForceVector3(const double* in_data, const unsigned int in_size);
        ForceVector3(const ForceVector3 & other);
        virtual ~ForceVector3();
    };

    /**
     * Method definitions
     */
    
    // constructors
    template <class ForceT, template <class MotionForceT> class ForceConversionsT>
    ForceVector3<ForceT, ForceConversionsT>::ForceVector3(): GeomVector3<ForceT, ForceConversionsT>()
    {}
    
    template <class ForceT, template <class MotionForceT> class ForceConversionsT>
    ForceVector3<ForceT, ForceConversionsT>::ForceVector3(const double* in_data, const unsigned int in_size): GeomVector3<ForceT, ForceConversionsT>(in_data, in_size)
    {}
    
    template <class ForceT, template <class MotionForceT> class ForceConversionsT>
    ForceVector3<ForceT, ForceConversionsT>::ForceVector3(const ForceVector3 & other): GeomVector3<ForceT, ForceConversionsT>(other)
    {}
    
    template <class ForceT, template <class MotionForceT> class ForceConversionsT>
    ForceVector3<ForceT, ForceConversionsT>::~ForceVector3()
    {}

    /**
     * Possible instanciations
     */
    //typedef ForceVector3<LinearForceVector3> ForceVector3LF;
    //typedef ForceVector3<AngularForceVector3> ForceVector3AF;
}

#endif /* IDYNTREE_FORCE_VECTOR_3_H */