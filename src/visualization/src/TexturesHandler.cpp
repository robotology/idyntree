// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

void iDynTree::TexturesHandler::draw(iDynTree::Environment &defaultEnvironment, iDynTree::Camera &defaultCamera, bool clearBuffers)
{
    if (m_textures.size() == 0)
    {
        return;
    }

    defaultEnvironment.m_envNode->setVisible(false); //Disable the visualizer environment

    for (auto t = m_textures.begin(); t != m_textures.end(); ++t)
    {
        if (t->second->shouldDraw)
        {
            t->second->textureEnvironment.m_envNode->setVisible(true); //Enable the texture environment
            bool clearThisTexture = clearBuffers || t->second->forceClear;
            // set render target texture
            m_irrDriver->setRenderTarget(t->second->irrTexture, clearThisTexture, clearThisTexture,
                                         t->second->textureEnvironment.m_backgroundColor.toSColor());

            m_irrDriver->setViewPort(irr::core::rect<irr::s32>(0, 0, t->second->irrTexture->getSize().Width, t->second->irrTexture->getSize().Height)); //workaround for http://irrlicht.sourceforge.net/forum/viewtopic.php?f=7&t=47004
            m_irrDriver->setViewPort(t->second->viewport); //Setting the viewport specified for the texture
            float width = t->second->viewport.LowerRightCorner.X - t->second->viewport.UpperLeftCorner.X;
            float height = t->second->viewport.LowerRightCorner.Y - t->second->viewport.UpperLeftCorner.Y;
            defaultCamera.setAspectRatio(width / height);

            // draw whole scene into render buffer
            m_sceneManager->drawAll();

            t->second->textureEnvironment.m_envNode->setVisible(false); //Disable the texture environment
        }
    }

    defaultEnvironment.m_envNode->setVisible(true); //Enable the visualizer environment
    // set back old render target
    // The buffer might have been distorted, so clear it
    m_irrDriver->setRenderTarget(0, clearBuffers, clearBuffers, defaultEnvironment.m_backgroundColor.toSColor());
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
