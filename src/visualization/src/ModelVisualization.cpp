/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Visualizer.h>

#ifdef IDYNTREE_USES_IRRLICHT
#include <irrlicht.h>
#include "IrrlichtUtils.h"
#endif

#include <cassert>

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

#ifdef IDYNTREE_USES_IRRLICHT
    irr::scene::ISceneNode * modelNode;
    std::vector<irr::scene::ISceneNode *> linkNodes;
    std::vector<irr::scene::ISceneNode *> linkFramesNodes;
    std::vector< std::vector<irr::scene::ISceneNode *> > geomNodes;
    irr::scene::ISceneManager * m_irrSmgr;

    void addModelGeometriesToSceneManager(const iDynTree::Model & model, const iDynTree::ModelSolidShapes & modelGeom);
    void updateLinkPositions(const iDynTree::LinkPositions & world_H_link);
#endif

    ModelVisualizationPimpl()
    {
        m_isValid = false;
        m_instanceName = "";
        enableContactWrenchesVisualization = false;
    }
};


#ifdef IDYNTREE_USES_IRRLICHT

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

    for(size_t linkIdx=0; linkIdx < model.getNrOfLinks(); linkIdx++)
    {
        this->linkNodes[linkIdx] = this->m_irrSmgr->addEmptySceneNode(this->modelNode);

        this->geomNodes[linkIdx].resize(modelGeom.linkSolidShapes[linkIdx].size());

        for(size_t geom=0; geom < modelGeom.linkSolidShapes[linkIdx].size(); geom++)
        {
            this->geomNodes[linkIdx][geom] = addGeometryToSceneManager(modelGeom.linkSolidShapes[linkIdx][geom],this->linkNodes[linkIdx],this->m_irrSmgr);
        }
    }
}

void ModelVisualization::ModelVisualizationPimpl::updateLinkPositions(const iDynTree::LinkPositions & world_H_link)
{
    for (LinkIndex linkIdx=0; linkIdx < static_cast<LinkIndex>(world_H_link.getNrOfLinks()); linkIdx++)
    {
        setWorldHNode(this->linkNodes[linkIdx],world_H_link(linkIdx));
    }
}

void ModelVisualization::ModelVisualizationPimpl::updateContactWrenches(const LinkContactWrenches& linkContacts)
{
    for (LinkIndex linkIdx=0; linkIdx < static_cast<LinkIndex>(linkContacts.getNrOfLinks()); linkIdx++)
    {
        for (int contactId=0; contactId < linkContacts.getNrOfContactsForLink(linkIdx); contactId++)
        {
            // createContactWrenchNodeIfNecessary(linkIdx,contactId)
            // enableContactWrenchNode(linkIdx,contactId);
            // updateContactWrench(linkIdx,contactId,linkContacts.contactWrench(linkIdx,contactId))
        }

        // Disable unused nodes
        for (int unusedNodeId=linkContacts.getNrOfContactsForLink(linkIdx);
                 unusedNodeId < contactWrenchesNodes[linkIdx].size();
                 unusedNodeId++)
        {
            // disableWrenchNode(linkIdx,unusedNodeId)
        }
    }
}

void ModelVisualization::ModelVisualizationPimpl::createContactWrenchNodeIfNecessary(LinkIndex idx, int contactId)
{

}

void ModelVisualization::ModelVisualizationPimpl::enableContactWrenchNode(LinkIndex idx, int contactId)
{

}

void ModelVisualization::ModelVisualizationPimpl::disableContactWrechNode(LinkIndex idx, int contactId)
{

}

void ModelVisualization::ModelVisualizationPimpl::updateContactWrench(LinkIndex idx, int contactId,
                                                                      const ContactWrench& contactWrench)
{
}



#endif

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
                              const std::string /*instanceName*/,
                              Visualizer& visualizer)
{
#ifdef IDYNTREE_USES_IRRLICHT
    this->pimpl->m_irrSmgr = visualizer.pimpl->m_irrSmgr;

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
#else
    reportError("Visualizer","init","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

Model& ModelVisualization::model()
{
    return this->pimpl->m_model;
}

bool ModelVisualization::setPositions(const Transform& world_H_base, const JointPosDoubleArray& jointPos)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( !jointPos.isConsistent(model()) )
    {
        reportError("ModelVisualization","setPositions","Input size mismatch.");
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

#else
    reportError("ModelVisualization","setPositions","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

bool ModelVisualization::setLinkPositions(const LinkPositions& linkPos)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( !linkPos.isConsistent(model()) )
    {
        reportError("ModelVisualization","setLinkPositions","Input size mismatch.");
        return false;
    }

    this->pimpl->updateLinkPositions(linkPos);
    return true;
#else
    reportError("ModelVisualization","setLinkPositions","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

bool ModelVisualization::visualizeContactWrenches(bool visualize)
{
    pimpl->enableContactWrenchesVisualization = visualize;

    return true;
}

bool ModelVisualization::setContactWrenches(const LinkContactWrenches& contactWrenches)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( contactWrenches.getNrOfLinks() != pimpl->m_model.getNrOfLinks() )
    {
        reportError("ModelVisualization","setContactWrenches","Input size mismatch.");
        return false;
    }

    if( pimpl->enableContactWrenchesVisualization )
    {
        pimpl->updateContactWrenches(contactWrenches);
    }

    return true;
#else
    reportError("ModelVisualization","setContactWrenches","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}



std::string ModelVisualization::getInstanceName()
{
    return this->pimpl->m_instanceName;
}


void ModelVisualization::close()
{

}

}
