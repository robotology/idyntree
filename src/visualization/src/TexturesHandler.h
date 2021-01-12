/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef IDYNTREE_TEXTUREHANDLER_H
#define IDYNTREE_TEXTUREHANDLER_H

#include <iDynTree/Visualizer.h>
#include <irrlicht.h>
#include "Texture.h"
#include "Camera.h"
#include <unordered_map>
#include <memory>

namespace iDynTree {

class TexturesHandler : public ITexturesHandler
{

    std::unordered_map<std::string, std::unique_ptr<Texture>> m_textures;
    irr::video::IVideoDriver* m_irrDriver{nullptr};
    irr::scene::ISceneManager *m_sceneManager{nullptr};
    bool m_areTexturesSupported{false};

public:

    /**
     * Destructor
     */
    virtual ~TexturesHandler();

    void init(irr::video::IVideoDriver* irrDriver,
              irr::scene::ISceneManager *sceneManager);

    void draw(Environment &defaultEnvironment, Camera& defaultCamera);

    /**
     * @brief Add a texture
     * @param The name of the texture
     * @param visualizerOptions The options for the texture
     * @return A ITexture pointer in case of success. A nullptr otherwise
     */
    virtual ITexture* add(const std::string& name, const VisualizerOptions& textureOptions = VisualizerOptions());

    /**
     * @brief Get a specific texture
     * @param The name of the texture to get.
     * @return the pointer to the texture. A nullptr if that texture does not exists.
     */
    virtual ITexture* get(const std::string& name);
};

}

#endif // IDYNTREE_TEXTUREHANDLER_H
