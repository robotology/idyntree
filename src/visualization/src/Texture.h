/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef IDYNTREE_TEXTURE_H
#define IDYNTREE_TEXTURE_H
#include <iDynTree/Visualizer.h>

#include "Environment.h"

#include <irrlicht.h>

namespace iDynTree
{

class Texture : public ITexture
{
public:


    irr::video::ITexture* irrTexture{nullptr};
    Environment textureEnvironment;

    void init(irr::video::IVideoDriver* irrDriver, irr::scene::ISceneManager *sceneManager, const std::string& name, const VisualizerOptions& textureOptions);

    virtual ~Texture();

    virtual IEnvironment& environment();

    virtual ColorViz getPixelColor(unsigned int width, unsigned int height) const;

    virtual bool getPixels(std::vector<PixelViz>& pixels) const;

};

}

#endif // IDYNTREE_TEXTURE_H
