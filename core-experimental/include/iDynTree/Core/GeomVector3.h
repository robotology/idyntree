/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_GEOM_VECTOR_3_H
#define IDYNTREE_GEOM_VECTOR_3_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>
#include <Eigen/Dense>

#define GEOMVECTOR3SEMANTICS_TEMPLATE_HDR \
template <class MotionForceTSemantics>
#define GEOMVECTOR3SEMANTICS_INSTANCE_HDR \
GeomVector3Semantics<MotionForceTSemantics>

#define GEOMVECTOR3_TEMPLATE_HDR \
template <class MotionForceT, class MotionForceAssociationsT, class MotionForceTSemantics>
#define GEOMVECTOR3_INSTANCE_HDR \
GeomVector3<MotionForceT, MotionForceAssociationsT, MotionForceTSemantics>

namespace iDynTree
{
    typedef Eigen::Matrix<double,3,1> Vector3d;
    typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

    class Rotation;

    /**
     * Template class providing the semantics for any geometric relation vector.
     */
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    class GeomVector3Semantics
    {
    protected:
        int body;
        int refBody;
        int coordinateFrame;
    
    public:
        /**
         * Constructors:
         */
        GeomVector3Semantics();
        GeomVector3Semantics(int _body, int _refBody, int _coordinateFrame);
        GeomVector3Semantics(const GeomVector3Semantics & other);
        ~GeomVector3Semantics();
        
        /**
         * Helpers
         */
        bool isUnknown();
        
        /**
         * Semantics operations
         * Compute the semantics of the result given the semantics of the operands.
         */
        bool changeCoordFrame(const Rotation & newCoordFrame);
        static bool compose(const MotionForceTSemantics & op1, const MotionForceTSemantics & op2, MotionForceTSemantics & result);
        static bool inverse(const MotionForceTSemantics & op, MotionForceTSemantics & result);

        bool dot(const MotionForceTSemantics & other) const;
    };

    /**
     * Template class providing the raw coordinates and semantics for any geometric relation vector.
     *
     * \ingroup iDynTreeCore
     *
     * A geometrical vector can be used to describe a motion or a force vector.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to motion and force vectors or to just provide an interface.
     *
     */
    GEOMVECTOR3_TEMPLATE_HDR
    class GeomVector3: public Vector3
    {
    protected:
        MotionForceTSemantics semantics;
        
    public:
        typedef GeomVector3<MotionForceT, MotionForceAssociationsT, MotionForceTSemantics> MotionForceTbase;
        
        /**
         * constructors
         */
        GeomVector3();
        GeomVector3(const double* in_data, const unsigned int in_size);
        GeomVector3(const GeomVector3 & other);
        virtual ~GeomVector3();
        
        /**
         * Getters & setters
         */
        const MotionForceTSemantics& getSemantics() const;
        void setSemantics(MotionForceTSemantics& _semantics);
        
        /**
         * Geometric operations
         */
        const MotionForceT & changeCoordFrame(const Rotation & newCoordFrame);
        static MotionForceT compose(const MotionForceTbase & op1, const MotionForceT & op2);
        static MotionForceT inverse(const MotionForceTbase & op);
        
        /**
         * dot product
         */
        double dot(const typename MotionForceAssociationsT::DualSpace & other) const;
        
        /**
         * overloaded operators
         */
        MotionForceT operator+(const MotionForceT &other) const;
        MotionForceT operator-(const MotionForceT &other) const;
        MotionForceT operator-() const;
    };

    /**
     * GeomVector3Semantics Method definitions ===================================
     */
    
    // constructors
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    GEOMVECTOR3SEMANTICS_INSTANCE_HDR::GeomVector3Semantics():
    body(UNKNOWN),
    refBody(UNKNOWN),
    coordinateFrame(UNKNOWN)
    {
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    GEOMVECTOR3SEMANTICS_INSTANCE_HDR::GeomVector3Semantics(int _body, int _refBody, int _coordinateFrame):
    body(_body),
    refBody(_refBody),
    coordinateFrame(_coordinateFrame)
    {
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    GEOMVECTOR3SEMANTICS_INSTANCE_HDR::GeomVector3Semantics(const GeomVector3Semantics & other):
    body(other.body),
    refBody(other.refBody),
    coordinateFrame(other.coordinateFrame)
    {
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    GEOMVECTOR3SEMANTICS_INSTANCE_HDR::~GeomVector3Semantics()
    {
    }

    // Helpers
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::isUnknown()
    {
        return (this->body == UNKNOWN && this->refBody == UNKNOWN && this->coordinateFrame == UNKNOWN);
    }
    
    // Semantics operations
    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::changeCoordFrame(const Rotation & newCoordFrame)
    {
        return true;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::compose(const MotionForceTSemantics & op1, const MotionForceTSemantics & op2, MotionForceTSemantics & result)
    {
        return true;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::inverse(const MotionForceTSemantics & op, MotionForceTSemantics & result)
    {
        return true;
    }

    GEOMVECTOR3SEMANTICS_TEMPLATE_HDR
    bool GEOMVECTOR3SEMANTICS_INSTANCE_HDR::dot(const MotionForceTSemantics & other) const
    {
        return true;
    }


    /**
     * GeomVector3 Method definitions ============================================
     */
    
    // constructors
    GEOMVECTOR3_TEMPLATE_HDR
    GEOMVECTOR3_INSTANCE_HDR::GeomVector3(): Vector3()
    {}
    
    GEOMVECTOR3_TEMPLATE_HDR
    GEOMVECTOR3_INSTANCE_HDR::GeomVector3(const double* in_data, const unsigned int in_size): Vector3(in_data, in_size)
    {}
    
    GEOMVECTOR3_TEMPLATE_HDR
    GEOMVECTOR3_INSTANCE_HDR::GeomVector3(const GeomVector3 & other): Vector3(other), semantics(other.semantics)
    {}
    
    GEOMVECTOR3_TEMPLATE_HDR
    GEOMVECTOR3_INSTANCE_HDR::~GeomVector3()
    {}
    
    // Getters & setters
    GEOMVECTOR3_TEMPLATE_HDR
    const MotionForceTSemantics& GEOMVECTOR3_INSTANCE_HDR::getSemantics() const
    {
        return semantics;
    }

    GEOMVECTOR3_TEMPLATE_HDR
    void GEOMVECTOR3_INSTANCE_HDR::setSemantics(MotionForceTSemantics& _semantics)
    {
        iDynTreeAssert(this->semantics.isUnknown());
        this->semantics = _semantics;
    }
    
    // Geometric operations
    GEOMVECTOR3_TEMPLATE_HDR
    const MotionForceT & GEOMVECTOR3_INSTANCE_HDR::changeCoordFrame(const Rotation & newCoordFrame)
    {
        iDynTreeAssert(semantics.changeCoordFrame(newCoordFrame.getSemantics()));
        
        Eigen::Map<Vector3d> thisData(this->data());
        Eigen::Map<const Matrix3dRowMajor> rotData(newCoordFrame.data());
        
        thisData = rotData*thisData;
        
        return *this;
    }
    
    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::compose(const MotionForceTbase & op1, const MotionForceT & op2)
    {
        MotionForceT result;
        
        iDynTreeAssert(GeomVector3Semantics<MotionForceTSemantics>::compose(op1.semantics, op2.semantics, result.semantics));
        
        Eigen::Map<const Vector3d> op1Data(op1.data());
        Eigen::Map<const Vector3d> op2Data(op2.data());
        Eigen::Map<Vector3d> resultData(result.data());
        
        resultData = op1Data + op2Data;
        
        return result;
    }
    
    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::inverse(const MotionForceTbase & op)
    {
        MotionForceT result;
        
        iDynTreeAssert(GeomVector3Semantics<MotionForceTSemantics>::inverse(op.semantics, result.semantics));
        
        Eigen::Map<const Vector3d> opData(op.data());
        Eigen::Map<Vector3d> resultData(result.data());
        
        resultData = -opData;
        
        return result;
    }
    
    // dot product
    GEOMVECTOR3_TEMPLATE_HDR
    double GEOMVECTOR3_INSTANCE_HDR::dot(const typename MotionForceAssociationsT::DualSpace & other) const
    {
        iDynTreeAssert(semantics.dot(other.semantics()));

        Eigen::Map<const Vector3d> otherData(other.data());
        Eigen::Map<const Vector3d> thisData(this->data());
        
        return thisData.dot(otherData);
    }

    // overloaded operators
    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::operator+(const MotionForceT &other) const
    {
        return compose(*this, other);
    }
    
    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::operator-(const MotionForceT &other) const
    {
        return compose(*this, inverse(other));
    }
    
    GEOMVECTOR3_TEMPLATE_HDR
    MotionForceT GEOMVECTOR3_INSTANCE_HDR::operator-() const
    {
        return inverse(*this);
    }
}

#endif /* IDYNTREE_GEOM_VECTOR_3_H */