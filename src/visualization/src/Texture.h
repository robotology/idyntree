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
    irr::video::IVideoDriver* irrDriver{nullptr};
    irr::core::rect<irr::s32> viewport;
    Environment textureEnvironment;
    bool shouldDraw{true};
    bool forceClear{false};

    void init(irr::video::IVideoDriver* irrDriverInput, irr::scene::ISceneManager *sceneManager, const std::string& name, const VisualizerOptions& textureOptions);

    virtual ~Texture();

    virtual IEnvironment& environment() override;

    virtual ColorViz getPixelColor(unsigned int width, unsigned int height) const override;

    virtual bool getPixels(std::vector<PixelViz>& pixels) const override;

    virtual bool drawToFile(const std::string filename="iDynTreeVisualizerTextureScreenshot.png") const override;

    virtual void enableDraw(bool enabled = true) override;

    virtual int width() const override;

    virtual int height() const override;

    virtual bool setSubDrawArea(int xOffsetFromTopLeft, int yOffsetFromTopLeft, int subImageWidth, int subImageHeight) override;

};

}

#endif // IDYNTREE_TEXTURE_H
