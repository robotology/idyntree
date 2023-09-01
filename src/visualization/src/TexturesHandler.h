// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
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

    void draw(Environment &defaultEnvironment, Camera& defaultCamera, bool clearBuffers);

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
