/**
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SOLID_SHAPES_H
#define IDYNTREE_SOLID_SHAPES_H

#include <cstdlib>
#include <string>
#include <vector>
#include <iDynTree/Core/Transform.h>

namespace iDynTree
{
    class Sphere;
    class Box;
    class Cylinder;
    class ExternalMesh;
    class Model;

    class SolidShape
    {
    public:
        virtual ~SolidShape()=0;
        virtual SolidShape* clone()=0;
        std::string name;
        Transform link_H_geometry;

        /**
         * Material of the geometry, encoded as a rgba vector.
         */
        Vector4 material;

        // To correctly wrap this objects in SWIG, we cannot rely on dynamic_cast .

        bool isSphere() const;
        bool isBox() const;
        bool isCylinder() const;
        bool isExternalMesh() const;

        Sphere* asSphere();
        Box *asBox();
        Cylinder* asCylinder();
        ExternalMesh* asExternalMesh();

        const Sphere* asSphere() const;
        const Box* asBox() const;
        const Cylinder* asCylinder() const;
        const ExternalMesh* asExternalMesh() const;
    };

    class Sphere: public SolidShape
    {
    public:
        virtual ~Sphere();
        virtual SolidShape* clone();
        double radius;
    };

    class Box: public SolidShape
    {
    public:
        virtual ~Box();
        virtual SolidShape* clone();
        double x;
        double y;
        double z;
    };

    class Cylinder: public SolidShape
    {
    public:
        virtual ~Cylinder();
        virtual SolidShape* clone();
        double length;
        double radius;
    };

    class ExternalMesh: public SolidShape
    {
    public:
        virtual ~ExternalMesh();
        virtual SolidShape* clone();
        std::string filename;
        iDynTree::Vector3 scale;
    };

    class ModelSolidShapes
    {
    private:
        ModelSolidShapes& copy(const ModelSolidShapes& other);

    public:
        ModelSolidShapes();
        ModelSolidShapes(const ModelSolidShapes& other);
        ModelSolidShapes& operator=(const ModelSolidShapes& other);
        void clear();
        ~ModelSolidShapes();
        void resize(size_t nrOfLinks);
        void resize(const Model& model);
        bool isConsistent(const Model & model) const;

        /**
         * Storage ot ModelSolidShapes.
         */
        std::vector< std::vector<SolidShape *> > linkSolidShapes;
    };

}

#endif
