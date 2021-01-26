/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
