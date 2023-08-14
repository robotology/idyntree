// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Texture.h"
#include "DummyImplementations.h"

void iDynTree::Texture::init(irr::video::IVideoDriver *irrDriverInput,
                             irr::scene::ISceneManager* sceneManager,
                             const std::string &name,
                             const VisualizerOptions &textureOptions)
{
    irrTexture = irrDriverInput->addRenderTargetTexture(irr::core::dimension2d<irr::u32>(textureOptions.winWidth, textureOptions.winHeight), name.c_str());
    viewport = {0, 0, textureOptions.winWidth, textureOptions.winHeight};
    textureEnvironment.init(sceneManager, textureOptions.rootFrameArrowsDimension);
    textureEnvironment.m_envNode->setVisible(false);
    irrDriver = irrDriverInput;
    irrDriver->grab();
}

iDynTree::Texture::~Texture()
{
    irrTexture = 0;
    if (irrDriver)
    {
        irrDriver->drop();
        irrDriver = nullptr;
    }
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

    irr::video::SColorf pixelIrrlichtFloat(pixelIrrlicht);
    pixelOut.r = pixelIrrlichtFloat.getRed();
    pixelOut.g = pixelIrrlichtFloat.getGreen();
    pixelOut.b = pixelIrrlichtFloat.getBlue();
    pixelOut.a = pixelIrrlichtFloat.getAlpha();

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
                irr::video::SColorf pixelIrrlichtFloat(pixelIrrlicht);
                pixels[i].width = width;
                pixels[i].height = height;
                pixels[i].r = pixelIrrlichtFloat.getRed();
                pixels[i].g = pixelIrrlichtFloat.getGreen();
                pixels[i].b = pixelIrrlichtFloat.getBlue();
                pixels[i].a = pixelIrrlichtFloat.getAlpha();
                ++i;
            }
        }

        irrTexture->unlock();
    }

    return true;
}

bool iDynTree::Texture::drawToFile(const std::string filename) const
{
    if (!irrTexture)
    {
        reportError("Texture","drawToFile","Cannot save the texture to file. It has not properly initialized.");
        return false;
    }

    auto textureDim = irrTexture->getSize();
    irr::video::IImage* const image = irrDriver->createImage(irrTexture,
                                                             irr::core::position2d<irr::s32>(0, 0),
                                                             textureDim);
    bool retValue = false;
    if (image) //should always be true, but you never know
    {
        //write screenshot to file
        if (!irrDriver->writeImageToFile(image, filename.c_str()))
        {
            std::stringstream ss;
            ss << "Impossible to write image file to " << filename;
            reportError("Texture","drawToFile",ss.str().c_str());
            retValue = false;
        }
        else
        {
            retValue = true;
        }

        //Don't forget to drop image since we don't need it anymore.
        image->drop();
    }
    else
    {
        reportError("Texture","drawToFile","Failed to create image from texture.");
        return false;
    }

    return retValue;
}

void iDynTree::Texture::enableDraw(bool enabled)
{
    shouldDraw = enabled;
    forceClear = enabled; //in case the texture has been enabled between one subDraw and the other.
}

int iDynTree::Texture::width() const
{
    if (!irrTexture)
    {
        return 0;
    }

    return irrTexture->getSize().Width;
}

int iDynTree::Texture::height() const
{
    if (!irrTexture)
    {
        return 0;
    }

    return irrTexture->getSize().Height;
}

bool iDynTree::Texture::setSubDrawArea(int xOffsetFromTopLeft, int yOffsetFromTopLeft, int subImageWidth, int subImageHeight)
{
    if (!irrTexture)
    {
        reportError("Texture","setSubDrawArea","Cannot get pixel color. The video texture has not been properly initialized.");
        return false;
    }

    if (xOffsetFromTopLeft + subImageWidth > width())
    {
        reportError("Texture","setSubDrawArea","The specified draw coordinates are out of bounds. The sum of the xOffsetFromTopLeft and width are greater than the texture width.");
        return false;
    }

    if (yOffsetFromTopLeft + subImageHeight > height())
    {
        reportError("Texture","setSubDrawArea","The specified draw coordinates are out of bounds. The sum of the yOffsetFromTopLeft and height are greater than the texture height.");
        return false;
    }

    if (subImageHeight <= 0)
    {
        return false;
    }

    viewport = {xOffsetFromTopLeft, yOffsetFromTopLeft, xOffsetFromTopLeft + subImageWidth, yOffsetFromTopLeft + subImageHeight};

    return true;
}
