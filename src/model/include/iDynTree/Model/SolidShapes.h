/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
        /**
         * True if the name is valid, false otherwise.
         */
        bool nameIsValid{false};
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
