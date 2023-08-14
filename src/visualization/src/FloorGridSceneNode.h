// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
    irr::video::SColor m_gridColor;

public:
    CFloorGridSceneNode(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id=-1);
    virtual void OnRegisterSceneNode();
    virtual void render();
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const;
    virtual irr::u32 getMaterialCount() const;
    virtual irr::video::SMaterial& getMaterial(irr::u32 i);
    void setGridColor(const irr::video::SColor& gridColor);
};

}

#endif
