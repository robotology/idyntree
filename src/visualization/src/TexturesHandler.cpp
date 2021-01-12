/*
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "TexturesHandler.h"

iDynTree::TexturesHandler::~TexturesHandler()
{
    if (m_irrDriver)
    {
        m_irrDriver->drop();
        m_irrDriver = nullptr;
    }

    if (m_sceneManager)
    {
        m_sceneManager->drop();
        m_sceneManager = nullptr;
    }
}

void iDynTree::TexturesHandler::init(irr::video::IVideoDriver *irrDriver,
                                     irr::scene::ISceneManager *sceneManager)
{
    m_irrDriver = irrDriver;
    m_irrDriver->grab();

    m_sceneManager = sceneManager;
    m_sceneManager->grab();

    m_areTexturesSupported = m_irrDriver->queryFeature(irr::video::EVDF_RENDER_TO_TARGET);
}

void iDynTree::TexturesHandler::draw(iDynTree::Environment &defaultEnvironment, iDynTree::Camera &defaultCamera)
{
    if (m_textures.size() == 0)
    {
        return;
    }

    defaultEnvironment.m_envNode->setVisible(false); //Disable the visualizer environment

    for (auto t = m_textures.begin(); t != m_textures.end(); ++t)
    {
        t->second->textureEnvironment.m_envNode->setVisible(true); //Enable the texture environment
        // set render target texture
       m_irrDriver->setRenderTarget(t->second->irrTexture, true, true,
                                    t->second->textureEnvironment.m_backgroundColor.toSColor());

        auto textureDims = t->second->irrTexture->getSize();

        defaultCamera.setAspectRatio(textureDims.Width/ (float)textureDims.Height);

        // draw whole scene into render buffer
        m_sceneManager->drawAll();

        t->second->textureEnvironment.m_envNode->setVisible(false); //Disable the texture environment

    }

    defaultEnvironment.m_envNode->setVisible(true); //Enable the visualizer environment
    // set back old render target
    // The buffer might have been distorted, so clear it
    m_irrDriver->setRenderTarget(0, true, true, defaultEnvironment.m_backgroundColor.toSColor());
}

iDynTree::ITexture *iDynTree::TexturesHandler::add(const std::string &name, const iDynTree::VisualizerOptions &textureOptions)
{
    if (!m_irrDriver || !m_sceneManager)
    {
        reportError("TexturesHandler", "add", "The handler has not been initialized yet. Call the init() method first.");
        return nullptr;
    }

    if (!m_areTexturesSupported)
    {
        reportError("TexturesHandler", "add", "The video driver does not support rendering to a target.");
        return nullptr;
    }

    if (m_textures.find(name) != m_textures.end())
    {
        std::stringstream ss;
        ss << "A texture with the name " << name << " already exists.";
        reportError("TexturesHandler", "add", ss.str().c_str());
        return nullptr;
    }

    std::unique_ptr<Texture> newTexture = std::make_unique<Texture>();
    newTexture->init(m_irrDriver, m_sceneManager, name, textureOptions);

    auto result = m_textures.insert(std::make_pair(name, std::move(newTexture)));

    if (!result.second)
    {
        reportError("TexturesHandler", "add", "Failed to insert new texture.");
        return nullptr;
    }

    return result.first->second.get();
}

iDynTree::ITexture *iDynTree::TexturesHandler::get(const std::string &name)
{
    auto result = m_textures.find(name);

    if (result == m_textures.end())
    {
        return nullptr;
    }

    return result->second.get();
}
