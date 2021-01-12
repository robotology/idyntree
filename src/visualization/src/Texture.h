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

    /**
     * @brief Initialize texture
     * @param Irrlicht driver
     * @param name Name of the texture
     * @param textureOptions Textures options
     */
    void init(irr::video::IVideoDriver* irrDriver, irr::scene::ISceneManager *sceneManager, const std::string& name, const VisualizerOptions& textureOptions);

    /**
     * Destructor
     */
    virtual ~Texture();

    /**
     * Return an interface to manipulate the texture environment.
     */
    virtual IEnvironment& environment();

    /**
     * @brief Get the color of the pixel at the given position in the additional texture.
     *
     * Remember to call draw() first.
     * @param width The width of the pixel
     * @param height The height of the pixel
     * @return The color of the pixel
     */
    virtual ColorViz getPixelColor(unsigned int width, unsigned int height) const;

    /**
     * @brief Get the pixels of the texture.
     *
     * Remember to call draw() first.
     * @param pixels The output pixels
     * @return True in case of success, false otherwise
     */
    virtual bool getPixels(std::vector<PixelViz>& pixels) const;

};

}

#endif // IDYNTREE_TEXTURE_H
