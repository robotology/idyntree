/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FLOORGRID_SCENENODE_H
#define IDYNTREE_FLOORGRID_SCENENODE_H

#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

namespace iDynTree
{

class CFloorGridSceneNode : public irr::scene::ISceneNode
{
private:
    irr::f32 m_size;
    irr::core::aabbox3d<irr::f32> m_box;
    irr::video::SMaterial m_dummyMaterial;

public:
    CFloorGridSceneNode(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id=-1);
    virtual void OnRegisterSceneNode();
    virtual void render();
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const;
    virtual irr::u32 getMaterialCount() const;
    virtual irr::video::SMaterial& getMaterial(irr::u32 i);
};

}

#endif
