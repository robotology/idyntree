/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "ModelVisualization.h"
#include "IrrlichtUtils.h"

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

namespace iDynTree
{

struct ModelVisualization::ModelVisualizationPimpl
{
    std::string m_instanceName;

    /**
     * Set to true if is a valid instance, false otherwise.
     */
    bool m_isValid;

    /**
     * Used model.
     */
    Model m_model;

    /**
     * Used traversal.
     */
    Traversal m_traversal;

    /**
     * Buffer for forward kinematics.
     */
    LinkPositions m_fwdKinBuffer;

    irr::scene::ISceneNode * modelNode;
    std::vector<irr::scene::ISceneNode *> linkNodes;
    std::vector<irr::scene::ISceneNode *> linkFramesNodes;
    std::vector< std::vector<irr::scene::ISceneNode *> > geomNodes;

    /**S
     * Cache of the original material of of the scene node
     */
    std::vector< std::vector< std::vector< irr::video::SMaterial > > > geomNodesNotTransparentMaterialCache;
    irr::scene::ISceneManager * m_irrSmgr;

    void addModelGeometriesToSceneManager(const iDynTree::Model & model, const iDynTree::ModelSolidShapes & modelGeom);
    void updateLinkPositions(const iDynTree::LinkPositions & world_H_link);


    ModelVisualizationPimpl()
    {
        m_isValid = false;
        m_instanceName = "";
    }
};


/**
 * Add  iDynTree::ModelGeometries to an irr::scene::ISceneManager*
 * We create a SceneGraph for the URDF model of this type: the root object
 * is the model, its child are the links, and the child of the links are the actual geometric objects
 * Note that in this case all the links are child of the model, and the scene graph does not mirror
 *  the kinematic graph of iDynTree Model
 */
void ModelVisualization::ModelVisualizationPimpl::addModelGeometriesToSceneManager(const iDynTree::Model & model,
                                                                                   const iDynTree::ModelSolidShapes & modelGeom)
{
    this->modelNode = this->m_irrSmgr->addEmptySceneNode();
    this->linkNodes.resize(model.getNrOfLinks());
    this->linkFramesNodes.resize(model.getNrOfLinks());
    this->geomNodes.resize(model.getNrOfLinks());
    this->geomNodesNotTransparentMaterialCache.resize(model.getNrOfLinks());

    for(size_t linkIdx=0; linkIdx < model.getNrOfLinks(); linkIdx++)
    {
        this->linkNodes[linkIdx] = this->m_irrSmgr->addEmptySceneNode(this->modelNode);

        this->geomNodes[linkIdx].resize(modelGeom.linkSolidShapes[linkIdx].size());
        this->geomNodesNotTransparentMaterialCache[linkIdx].resize(modelGeom.linkSolidShapes[linkIdx].size());

        for(size_t geom=0; geom < modelGeom.linkSolidShapes[linkIdx].size(); geom++)
        {
            this->geomNodes[linkIdx][geom] = addGeometryToSceneManager(modelGeom.linkSolidShapes[linkIdx][geom],this->linkNodes[linkIdx],this->m_irrSmgr);

            if( this->geomNodes[linkIdx][geom] )
            {
                 this->geomNodesNotTransparentMaterialCache[linkIdx][geom].resize(this->geomNodes[linkIdx][geom]->getMaterialCount());

                 for( size_t mat = 0; mat < this->geomNodesNotTransparentMaterialCache[linkIdx][geom].size(); mat++)
                 {
                     this->geomNodesNotTransparentMaterialCache[linkIdx][geom][mat] = this->geomNodes[linkIdx][geom]->getMaterial(mat);
                 }
            }
        }
    }
}

void ModelVisualization::ModelVisualizationPimpl::updateLinkPositions(const iDynTree::LinkPositions & world_H_link)
{
    for(size_t linkIdx=0; linkIdx < world_H_link.getNrOfLinks(); linkIdx++)
    {
        setWorldHNode(this->linkNodes[linkIdx],world_H_link(linkIdx));
    }
}

ModelVisualization::ModelVisualization():
    pimpl(new ModelVisualizationPimpl())
{
}

ModelVisualization::~ModelVisualization()
{
    this->close();
    if( pimpl )
    {
        delete pimpl;
        pimpl = 0;
    }
}


ModelVisualization::ModelVisualization(const ModelVisualization& /*other*/)
{
    assert(false);
}

ModelVisualization& ModelVisualization::operator=(const ModelVisualization& /*other*/)
{
    assert(false);
    return *this;
}

bool ModelVisualization::init(const Model& model,
                              const std::string instanceName,
                              irr::scene::ISceneManager * sceneManager)
{
    // Check if the visual of the models are consisten with the rest of the model
    if( !model.visualSolidShapes().isConsistent(model) )
    {
        reportError("ModelVisualization","init","Impossible to use load model, as the visual solid shapes of the model are not consistent with the model itself.");
        return false;
    }
    this->pimpl->m_instanceName = instanceName;

    this->pimpl->m_irrSmgr = sceneManager;

    // Copy the model and create a traversal from the default base
    this->pimpl->m_model = model;
    this->pimpl->m_model.computeFullTreeTraversal(this->pimpl->m_traversal);

    // Resize fwd kinematics buffer
    this->pimpl->m_fwdKinBuffer.resize(this->pimpl->m_model);

    // Create model in the scene, using visual solidShapes
    pimpl->addModelGeometriesToSceneManager(model,model.visualSolidShapes());

    // Set the initial position of the model
    Transform world_H_base = Transform::Identity();
    JointPosDoubleArray jointPos(model);
    jointPos.zero();

    this->setPositions(world_H_base,jointPos);

    pimpl->m_isValid = true;

    return true;
}

Model& ModelVisualization::model()
{
    return this->pimpl->m_model;
}

bool ModelVisualization::setPositions(const Transform& world_H_base, const VectorDynSize& jointPos)
{
    if( (jointPos.size() != model().getNrOfPosCoords()) )
    {
        std::stringstream ss;
        ss << "Input size mismatch: model internal position coords " << model().getNrOfPosCoords() << " provided vector " << jointPos.size();
        reportError("ModelVisualization","setPositions",ss.str().c_str());
        return false;
    }

    // Compute fwd kinematics
    bool ok = ForwardPositionKinematics(model(), this->pimpl->m_traversal,
                                        world_H_base, jointPos,
                                        this->pimpl->m_fwdKinBuffer);

    if( ok )
    {
        this->pimpl->updateLinkPositions(this->pimpl->m_fwdKinBuffer);
    }
    else
    {
        reportError("ModelVisualization","setPositions","Forward kinematics error.");
    }

    return ok;
}

bool ModelVisualization::setLinkPositions(const LinkPositions& linkPos)
{
    if( !linkPos.isConsistent(model()) )
    {
        reportError("ModelVisualization","setLinkPositions","Input size mismatch.");
        return false;
    }

    this->pimpl->updateLinkPositions(linkPos);
    return true;
}

std::string ModelVisualization::getInstanceName()
{
    return this->pimpl->m_instanceName;
}


void ModelVisualization::close()
{

}

std::vector<std::string> ModelVisualization::getFeatures()
{
    std::vector<std::string> ret;
    ret.push_back("wireframe");
    ret.push_back("transparent");

    return ret;
}

bool ModelVisualization::setFeatureVisibility(const std::string elementKey, bool isVisible)
{
    bool retValue = false;
    if( elementKey == "wireframe"  )
    {
        this->setWireframeVisibility(isVisible);
        retValue = true;
    }

    if( elementKey == "transparent"  )
    {
        this->setTransparent(isVisible);
        retValue = true;
    }

    return retValue;
}

void ModelVisualization::setWireframeVisibility(bool isVisible)
{
    for(size_t linkIdx=0; linkIdx < pimpl->geomNodes.size(); linkIdx++)
    {
        for(size_t geom=0; geom < pimpl->geomNodes[linkIdx].size(); geom++)
        {
            if( pimpl->geomNodes[linkIdx][geom] )
            {
                irr::scene::ISceneNode * geomNode = pimpl->geomNodes[linkIdx][geom];
                std::vector< irr::video::SMaterial > & materialCache = pimpl->geomNodesNotTransparentMaterialCache[linkIdx][geom];

                for( size_t mat = 0; mat < geomNode->getMaterialCount(); mat++)
                {
                    geomNode->getMaterial(mat).setFlag(irr::video::EMF_WIREFRAME,isVisible);
                    materialCache[mat].setFlag(irr::video::EMF_WIREFRAME,isVisible);
                }
            }
        }
    }
}

void ModelVisualization::setTransparent(bool isTransparent)
{
    irr::u32 alphaValue = 0;
    for(size_t linkIdx=0; linkIdx < pimpl->geomNodes.size(); linkIdx++)
    {
        for(size_t geom=0; geom < pimpl->geomNodes[linkIdx].size(); geom++)
        {
            if( pimpl->geomNodes[linkIdx][geom] )
            {
                irr::scene::ISceneNode * geomNode = pimpl->geomNodes[linkIdx][geom];
                std::vector< irr::video::SMaterial > & materialCache = pimpl->geomNodesNotTransparentMaterialCache[linkIdx][geom];

                for( size_t mat = 0; mat < geomNode->getMaterialCount(); mat++)
                {
                    // If we need to set the model to being transparent, we modify the material to have an alpha of at least
                    // 0.5 on all light components
                    if( isTransparent )
                    {
                        irr::video::SMaterial geomMat = geomNode->getMaterial(mat);
                        geomMat.MaterialType = irr::video::EMT_TRANSPARENT_ADD_COLOR;
                        geomMat.AmbientColor.setAlpha(std::min(alphaValue,geomMat.AmbientColor.getAlpha()));
                        geomMat.DiffuseColor.setAlpha(std::min(alphaValue,geomMat.DiffuseColor.getAlpha()));
                        geomMat.SpecularColor.setAlpha(std::min(alphaValue,geomMat.SpecularColor.getAlpha()));
                        geomMat.DiffuseColor.setAlpha(std::min(alphaValue,geomMat.DiffuseColor.getAlpha()));
                        geomNode->getMaterial(mat) = geomMat;
                    }
                    else
                    {
                        // otherwise we just restore the cached value of the material
                        geomNode->getMaterial(mat) = materialCache[mat];
                    }

                }
            }
        }
    }
}

}
