/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/SolidShapes.h>
#include <iDynTree/Model/Model.h>

namespace iDynTree
{
    SolidShape::~SolidShape()
    {
    }

    bool SolidShape::isSphere() const
    {
        return (dynamic_cast<const Sphere*>(this) != 0);
    }

    Sphere* SolidShape::asSphere()
    {
        return dynamic_cast<Sphere*>(this);
    }

    const Sphere* SolidShape::asSphere() const
    {
        return dynamic_cast<const Sphere*>(this);
    }

    bool SolidShape::isCylinder() const
    {
        return (dynamic_cast<const Cylinder*>(this) != 0);
    }

    Cylinder * SolidShape::asCylinder()
    {
        return dynamic_cast<Cylinder*>(this);
    }

    const Cylinder * SolidShape::asCylinder() const
    {
        return dynamic_cast<const Cylinder*>(this);
    }

    bool SolidShape::isBox() const
    {
        return (dynamic_cast<const Box*>(this) != 0);
    }

    Box * SolidShape::asBox()
    {
        return dynamic_cast<Box*>(this);
    }

    const Box * SolidShape::asBox() const
    {
        return dynamic_cast<const Box*>(this);
    }

    bool SolidShape::isExternalMesh() const
    {
        return (dynamic_cast<const ExternalMesh*>(this) != 0);
    }

    ExternalMesh * SolidShape::asExternalMesh()
    {
        return dynamic_cast<ExternalMesh*>(this);
    }

    const ExternalMesh * SolidShape::asExternalMesh() const
    {
        return dynamic_cast<const ExternalMesh*>(this);
    }


    Sphere::~Sphere()
    {
    }

    SolidShape* Sphere::clone()
    {
        return static_cast<SolidShape*>(new Sphere(*this));
    }

    Box::~Box()
    {
    }

    SolidShape* Box::clone()
    {
        return static_cast<SolidShape*>(new Box(*this));
    }

    Cylinder::~Cylinder()
    {
    }

    SolidShape* Cylinder::clone()
    {
        return static_cast<SolidShape*>(new Cylinder(*this));
    }

    ExternalMesh::~ExternalMesh()
    {
    }

    SolidShape* ExternalMesh::clone()
    {
        return static_cast<SolidShape*>(new ExternalMesh(*this));
    }

    void ModelSolidShapes::clear()
    {
        for(size_t link = 0; link < linkSolidShapes.size(); link++)
        {
            for(size_t geom = 0; geom < linkSolidShapes[link].size(); geom++)
            {
                delete linkSolidShapes[link][geom];
                linkSolidShapes[link][geom] = 0;
            }
        }

        linkSolidShapes.resize(0);
    }

    ModelSolidShapes& ModelSolidShapes::copy(const ModelSolidShapes& other)
    {
        clear();
        this->linkSolidShapes.resize(other.linkSolidShapes.size());
        for(size_t link = 0; link < other.linkSolidShapes.size(); link++)
        {
            this->linkSolidShapes[link].resize(other.linkSolidShapes[link].size());
            for(size_t geom = 0; geom < other.linkSolidShapes[link].size(); geom++)
            {
                this->linkSolidShapes[link][geom] = other.linkSolidShapes[link][geom]->clone();
            }
        }

        return *this;
    }

    void ModelSolidShapes::resize(const Model& model)
    {
        this->resize(model.getNrOfLinks());
    }

    void ModelSolidShapes::resize(const size_t nrOfLinks)
    {
        clear();
        this->linkSolidShapes.resize(nrOfLinks);
    }

    ModelSolidShapes::ModelSolidShapes()
    {
    }

    ModelSolidShapes::ModelSolidShapes(const ModelSolidShapes& other)
    {
        copy(other);
    }

    ModelSolidShapes& ModelSolidShapes::operator=(const ModelSolidShapes& other)
    {
        return copy(other);
    }

    ModelSolidShapes::~ModelSolidShapes()
    {
        this->clear();
    }
}
