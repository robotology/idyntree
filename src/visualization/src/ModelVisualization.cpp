/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "ModelVisualization.h"
#include "JetsVisualization.h"
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

    /**
     * Empty SceneNode representing the link frames.
     */
    std::vector<irr::scene::ISceneNode *> linkNodes;

    /**
     * Empty SceneNodes representing the frames, both the principal link frames and the added ones.
     */
    std::vector<irr::scene::ISceneNode *> frameNodes;

    std::vector<irr::scene::ISceneNode *> linkFramesNodes;
    std::vector< std::vector<irr::scene::ISceneNode *> > geomNodes;

    /**
     * Cache of the original material of of the scene node
     */
    std::vector< std::vector< std::vector< irr::video::SMaterial > > > geomNodesNotTransparentMaterialCache;
    irr::scene::ISceneManager * m_irrSmgr;

    void addModelGeometriesToSceneManager(const iDynTree::Model & model, const iDynTree::ModelSolidShapes & modelGeom);
    void updateLinkPositions(const iDynTree::LinkPositions & world_H_link);

    /**
     * JetsVisualization helper class
     */
    JetsVisualization m_jets;


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
    this->frameNodes.resize(model.getNrOfFrames());
    this->linkFramesNodes.resize(model.getNrOfLinks());
    this->geomNodes.resize(model.getNrOfLinks());
    this->geomNodesNotTransparentMaterialCache.resize(model.getNrOfLinks());

    for(size_t linkIdx=0; linkIdx < model.getNrOfLinks(); linkIdx++)
    {
        this->linkNodes[linkIdx] = this->m_irrSmgr->addEmptySceneNode(this->modelNode);
        this->frameNodes[linkIdx] = this->linkNodes[linkIdx];

        this->geomNodes[linkIdx].resize(modelGeom.getLinkSolidShapes()[linkIdx].size());
        this->geomNodesNotTransparentMaterialCache[linkIdx].resize(modelGeom.getLinkSolidShapes()[linkIdx].size());

        for(size_t geom=0; geom < modelGeom.getLinkSolidShapes()[linkIdx].size(); geom++)
        {
            this->geomNodes[linkIdx][geom] = addGeometryToSceneManager(modelGeom.getLinkSolidShapes()[linkIdx][geom],this->linkNodes[linkIdx],this->m_irrSmgr);

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

    // Add also all the additional frames of each link
    for(size_t frameIdx=model.getNrOfLinks(); frameIdx < model.getNrOfFrames(); frameIdx++ )
    {
        LinkIndex parentLinkIdx = model.getFrameLink(frameIdx);
        this->frameNodes[frameIdx] = this->m_irrSmgr->addEmptySceneNode(this->linkNodes[parentLinkIdx]);

        // Set the position of the added frame w.r.t. to the parent link, that will remain constant
        this->frameNodes[frameIdx]->setPosition(idyntree2irr_pos(model.getFrameTransform(frameIdx).getPosition()));
        this->frameNodes[frameIdx]->setRotation(idyntree2irr_rpy(model.getFrameTransform(frameIdx).getRotation().asRPY()));
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

    // Initialize the jets visualizer
    this->pimpl->m_jets.init(this->pimpl->m_irrSmgr,this,&(this->pimpl->frameNodes));

    pimpl->m_isValid = true;

    return true;
}

Model& ModelVisualization::model()
{
    return this->pimpl->m_model;
}

Transform ModelVisualization::getWorldModelTransform()
{
    Transform w_H_b;
    irr::core::matrix4 relativeTransform(this->pimpl->modelNode->getRelativeTransformation());
    w_H_b = irr2idyntree_trans(relativeTransform);
    return w_H_b;
}

Transform ModelVisualization::getWorldLinkTransform(const LinkIndex& linkIndex)
{
    if (linkIndex < 0 || linkIndex >= pimpl->geomNodes.size())
    {
        reportError("ModelVisualization","getWorldToLinkTransorm", "invalid link index. returning identity transform");
        return Transform::Identity();
    }

    Transform w_H_link;
    irr::core::matrix4 relativeLinkTransform(this->pimpl->linkNodes[linkIndex]->getRelativeTransformation());
    w_H_link = irr2idyntree_trans(relativeLinkTransform);
    return w_H_link;
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

void ModelVisualization::setModelVisibility(const bool isVisible)
{
    if( pimpl->modelNode )
    {
        pimpl->modelNode->setVisible(isVisible);
    }
}

bool ModelVisualization::setLinkColor(const LinkIndex& linkIndex, const ColorViz& linkColor)
{
    if (linkIndex < 0 || linkIndex >= pimpl->geomNodes.size())
    {
        reportError("ModelVisualization","setLinkColor", "invalid link index");
        return false;
    }

    irr::video::SColor col = idyntree2irrlicht(linkColor).toSColor();
    for(size_t geom=0; geom < pimpl->geomNodes[linkIndex].size(); geom++)
    {
        if( pimpl->geomNodes[linkIndex][geom] )
        {
            irr::scene::ISceneNode * geomNode = pimpl->geomNodes[linkIndex][geom];

            for( size_t mat = 0; mat < geomNode->getMaterialCount(); mat++)
            {
                irr::video::SMaterial geomMat = geomNode->getMaterial(mat);

                // R
                geomMat.AmbientColor.setRed(col.getRed());
                geomMat.DiffuseColor.setRed(col.getRed());
                geomMat.SpecularColor.setRed(col.getRed());
                geomMat.EmissiveColor.setRed(col.getRed());

                // G
                geomMat.AmbientColor.setGreen(col.getGreen());
                geomMat.DiffuseColor.setGreen(col.getGreen());
                geomMat.SpecularColor.setGreen(col.getGreen());
                geomMat.EmissiveColor.setGreen(col.getGreen());

                // B
                geomMat.AmbientColor.setBlue(col.getBlue());
                geomMat.DiffuseColor.setBlue(col.getBlue());
                geomMat.SpecularColor.setBlue(col.getBlue());
                geomMat.EmissiveColor.setBlue(col.getBlue());

                geomNode->getMaterial(mat) = geomMat;
            }
        }
    }
    return true;
}

bool ModelVisualization::resetLinkColor(const LinkIndex& linkIndex)
{
    if (linkIndex < 0 || linkIndex >= pimpl->geomNodes.size())
    {
        reportError("ModelVisualization","resetLinkColor", "invalid link index");
        return false;
    }

    for(size_t geom=0; geom < pimpl->geomNodes[linkIndex].size(); geom++)
    {
        if( pimpl->geomNodes[linkIndex][geom] )
        {
            irr::scene::ISceneNode * geomNode = pimpl->geomNodes[linkIndex][geom];
            std::vector< irr::video::SMaterial > & materialCache = pimpl->geomNodesNotTransparentMaterialCache[linkIndex][geom];

            for( size_t mat = 0; mat < geomNode->getMaterialCount(); mat++)
            {
                irr::video::SMaterial geomMat = geomNode->getMaterial(mat);

                // R
                geomMat.AmbientColor.setRed(materialCache[mat].AmbientColor.getRed());
                geomMat.DiffuseColor.setRed(materialCache[mat].DiffuseColor.getRed());
                geomMat.SpecularColor.setRed(materialCache[mat].SpecularColor.getRed());
                geomMat.EmissiveColor.setRed(materialCache[mat].EmissiveColor.getRed());

                // G
                geomMat.AmbientColor.setGreen(materialCache[mat].AmbientColor.getGreen());
                geomMat.DiffuseColor.setGreen(materialCache[mat].DiffuseColor.getGreen());
                geomMat.SpecularColor.setGreen(materialCache[mat].SpecularColor.getGreen());
                geomMat.EmissiveColor.setGreen(materialCache[mat].EmissiveColor.getGreen());

                // B
                geomMat.AmbientColor.setBlue(materialCache[mat].AmbientColor.getBlue());
                geomMat.DiffuseColor.setBlue(materialCache[mat].DiffuseColor.getBlue());
                geomMat.SpecularColor.setBlue(materialCache[mat].SpecularColor.getBlue());
                geomMat.EmissiveColor.setBlue(materialCache[mat].EmissiveColor.getBlue());

                geomNode->getMaterial(mat) = geomMat;
            }
        }
    }
   return true;
}


void ModelVisualization::setModelColor(const ColorViz& modelColor)
{
    for(size_t linkIdx=0; linkIdx < pimpl->geomNodes.size(); linkIdx++)
    {
        this->setLinkColor(linkIdx, modelColor);
    }
}

void ModelVisualization::resetModelColor()
{
    for(size_t linkIdx=0; linkIdx < pimpl->geomNodes.size(); linkIdx++)
    {
        this->resetLinkColor(linkIdx);
    }
}

std::vector< std::string > ModelVisualization::getLinkNames()
{
    std::vector< std::string > ret;

    for( size_t i=0; i < model().getNrOfLinks(); i++)
    {
        ret.push_back(model().getLinkName(i));
    }

    return ret;
}

bool ModelVisualization::setLinkVisibility(const std::string& linkName, bool isVisible)
{
    LinkIndex linkIdx = model().getLinkIndex(linkName);

    if( linkIdx == LINK_INVALID_INDEX )
    {
        std::stringstream ss;
        ss << "Unknown link " << linkName;
        reportError("ModelVisualization","setLinkVisibility",ss.str().c_str());
        return false;
    }

    if( pimpl->linkNodes[linkIdx] )
    {
        pimpl->linkNodes[linkIdx]->setVisible(isVisible);
        return true;
    }

    return false;
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

bool ModelVisualization::setFeatureVisibility(const std::string& elementKey, bool isVisible)
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

                for( size_t mat = 0; mat < geomNode->getMaterialCount(); mat++)
                {
                    geomNode->getMaterial(mat).setFlag(irr::video::EMF_WIREFRAME,isVisible);
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
                        geomMat.EmissiveColor.setAlpha(std::min(alphaValue,geomMat.EmissiveColor.getAlpha()));
                        geomNode->getMaterial(mat) = geomMat;
                    }
                    else
                    {
                        // otherwise we just restore the cached value of alpha
                        irr::video::SMaterial geomMat = geomNode->getMaterial(mat);
                        geomMat.MaterialType = materialCache[mat].MaterialType;
                        geomMat.AmbientColor.setAlpha(materialCache[mat].AmbientColor.getAlpha());
                        geomMat.DiffuseColor.setAlpha(materialCache[mat].DiffuseColor.getAlpha());
                        geomMat.SpecularColor.setAlpha(materialCache[mat].SpecularColor.getAlpha());
                        geomMat.EmissiveColor.setAlpha(materialCache[mat].EmissiveColor.getAlpha());
                        geomNode->getMaterial(mat) = geomMat;
                    }

                }
            }
        }
    }
}

IJetsVisualization & ModelVisualization::jets()
{
    return this->pimpl->m_jets;
}

}
