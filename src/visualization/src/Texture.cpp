/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "Texture.h"
#include "DummyImplementations.h"

void iDynTree::Texture::init(irr::video::IVideoDriver *irrDriver,
                             irr::scene::ISceneManager* sceneManager,
                             const std::string &name,
                             const VisualizerOptions &textureOptions)
{
    irrTexture = irrDriver->addRenderTargetTexture(irr::core::dimension2d<irr::u32>(textureOptions.winWidth, textureOptions.winHeight), name.c_str());
    textureEnvironment.init(sceneManager, textureOptions.rootFrameArrowsDimension);
    textureEnvironment.m_envNode->setVisible(false);
}

iDynTree::Texture::~Texture()
{
    irrTexture = 0;
}

iDynTree::IEnvironment &iDynTree::Texture::environment()
{
    return textureEnvironment;
}

iDynTree::ColorViz iDynTree::Texture::getPixelColor(unsigned int width, unsigned int height) const
{
    ColorViz pixelOut;
    irr::video::SColor pixelIrrlicht = irr::video::SColor(0, 0, 0, 0);

    if (!irrTexture)
    {
        reportError("Texture","getTexturePixelColor","Cannot get pixel color. The video texture has not been properly initialized.");
        return pixelOut;
    }

    auto textureDim = irrTexture->getSize();

    if ( width >= textureDim.Width || height >= textureDim.Height)
    {
        std::stringstream ss;
        ss << "The requested pixel is out of bounds. Requested (" << width << ", " << height
           << "). Picture dimensions: (" << textureDim.Width  << ", " << textureDim.Height << ").";
        reportError("Visualizer", "getTexturePixelColor", ss.str().c_str());
        return pixelOut;
    }

    auto pitch = irrTexture->getPitch();
    auto format = irrTexture->getColorFormat();
    auto bytes = irr::video::IImage::getBitsPerPixelFromFormat(format) / 8;

    unsigned char* buffer = (unsigned char*) irrTexture->lock(irr::video::E_TEXTURE_LOCK_MODE::ETLM_READ_ONLY);
    if (buffer)
    {
        pixelIrrlicht = irr::video::SColor(*(unsigned int*)(buffer + (height * pitch) + (width * bytes)));
        irrTexture->unlock();
    }

    pixelOut.r = pixelIrrlicht.getRed();
    pixelOut.g = pixelIrrlicht.getGreen();
    pixelOut.b = pixelIrrlicht.getBlue();
    pixelOut.a = pixelIrrlicht.getAlpha();

    return pixelOut;
}

bool iDynTree::Texture::getPixels(std::vector<iDynTree::PixelViz> &pixels) const
{
    irr::video::SColor pixelIrrlicht = irr::video::SColor(0, 0, 0, 0);

    if (!irrTexture)
    {
        reportError("Texture","getTexturePixels","Cannot get pixel color. The video texture has not been properly initialized.");
        return false;
    }

    auto textureDim = irrTexture->getSize();

    pixels.resize(textureDim.Width * textureDim.Height);

    auto pitch = irrTexture->getPitch();
    auto format = irrTexture->getColorFormat();
    auto bytes = irr::video::IImage::getBitsPerPixelFromFormat(format) / 8;

    unsigned char* buffer = (unsigned char*) irrTexture->lock(irr::video::E_TEXTURE_LOCK_MODE::ETLM_READ_ONLY);
    if (buffer)
    {
        size_t i = 0;
        for (size_t width = 0; width < textureDim.Width; ++width)
        {
            for (size_t height = 0; height < textureDim.Height; ++height)
            {
                pixelIrrlicht = irr::video::SColor(*(unsigned int*)(buffer + (height * pitch) + (width * bytes)));
                pixels[i].width = width;
                pixels[i].height = height;
                pixels[i].r = pixelIrrlicht.getRed();
                pixels[i].g = pixelIrrlicht.getGreen();
                pixels[i].b = pixelIrrlicht.getBlue();
                pixels[i].a = pixelIrrlicht.getAlpha();
                ++i;
            }
        }

        irrTexture->unlock();
    }

    return true;
}
