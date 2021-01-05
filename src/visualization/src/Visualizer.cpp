/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Visualizer.h>

#ifdef IDYNTREE_USES_IRRLICHT
#include <irrlicht.h>
#include "IrrlichtUtils.h"
#include "Camera.h"
#include "Environment.h"
#include "ModelVisualization.h"
#include "VectorsVisualization.h"
#include "CameraAnimator.h"
#endif

#include "DummyImplementations.h"

#include <cassert>

namespace iDynTree
{

ICamera::~ICamera()
{
}

IEnvironment::~IEnvironment()
{
}

IJetsVisualization::~IJetsVisualization()
{

}

IVectorsVisualization::~IVectorsVisualization()
{
}

IModelVisualization::~IModelVisualization()
{
}

ILight::~ILight()
{
}


ColorViz::ColorViz(): r(1.0), g(1.0), b(1.0), a(1.0)
{
    // default color is white
}

ColorViz::ColorViz(float _r, float _g, float _b, float _a): r(_r), g(_g), b(_b), a(_a)
{
}

ColorViz::ColorViz(const Vector4& rgba): r(rgba(0)), g(rgba(1)), b(rgba(2)), a(rgba(3))
{

}

struct Visualizer::VisualizerPimpl
{
    /**
     * True if init has been called.
     */
    bool m_isInitialized;

    /**
     * Invalid model visualization, useful to return in case of error.
     */
    DummyModelVisualization m_invalidModelViz;

    /**
     * Last FPS measured.
     */
    int lastFPS;

#ifdef IDYNTREE_USES_IRRLICHT
    /**
     * Collection of model visualization.
     */
    std::vector<ModelVisualization*> m_modelViz;

    /**
     * Irrlicht device used by the visualizer.
     */
    irr::IrrlichtDevice* m_irrDevice;

    /**
     * Irrlicht scene manager.
     */
    irr::scene::ISceneManager* m_irrSmgr;

    /**
     * Irrlicht video driver.
     */
    irr::video::IVideoDriver* m_irrDriver;

    /**
     * Additional texture background color.
     */
    irr::video::SColorf m_irrTextureBackgroundColor;

    /**
     * Additional texture to get pixels' color.
     */
    irr::video::ITexture* m_irrTexture;

    /**
     * Camera used by the visualization.
     */
    Camera m_camera;

    /**
     * Environment used by the visualization.
     */
    Environment m_environment;

    /**
     * Vectors visualization
     */
    VectorsVisualization m_vectors;

#else
    DummyCamera m_camera;
    DummyEnvironment m_environment;
    DummyVectorsVisualization m_invalidVectors;
#endif

    VisualizerPimpl()
    {
        m_isInitialized = false;
        lastFPS = -1;

#ifdef IDYNTREE_USES_IRRLICHT
        m_modelViz.resize(0);
        m_irrDevice  = 0;
        m_irrSmgr    = 0;
        m_irrDriver  = 0;
        m_irrTexture = 0;
#endif
    }
};


Visualizer::Visualizer(const Visualizer& /*other*/)
{
    assert(false);
}

Visualizer& Visualizer::operator=(const Visualizer& /*other*/)
{
    assert(false);
    return *this;
}

Visualizer::Visualizer():
    pimpl(new VisualizerPimpl())
{
}

Visualizer::~Visualizer()
{
    close();

    delete pimpl;
    pimpl = 0;
}

bool Visualizer::init(const VisualizerOptions &visualizerOptions, const VisualizerOptions &textureOptions)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( pimpl->m_isInitialized )
    {
        reportWarning("Visualizer","init","Visualier already initialized, call close() to close it to open it again.");
        return false;
    }

    irr::SIrrlichtCreationParameters irrDevParams;

    irrDevParams.DriverType = irr::video::EDT_OPENGL;
    irrDevParams.WindowSize = irr::core::dimension2d<irr::u32>(visualizerOptions.winWidth, visualizerOptions.winHeight);
    irrDevParams.WithAlphaChannel = true;

    if( visualizerOptions.verbose )
    {
        reportWarning("Visualizer","init","verbose flag found, enabling verbose output in Visualizer");
        irrDevParams.LoggingLevel = irr::ELL_DEBUG;
    }

    pimpl->m_irrDevice = 0;
    pimpl->m_irrDevice = irr::createDeviceEx(irrDevParams);


    if (pimpl->m_irrDevice == 0)
    {
        reportError("Visualizer","init","Impossible to load irrlicht device");
        return false; // could not create selected driver.
    }

    // Checking the irrDevice
    if (pimpl->m_irrDevice == (void*)1)
    {
        reportError("Visualizer","init","Impossible to load irrlicht device, createDevice returned a 1 pointer");
        pimpl->m_irrDevice = 0;
        return false; // could not create selected driver.
    }

    pimpl->m_irrSmgr = pimpl->m_irrDevice->getSceneManager();

    // Get video driver
    pimpl->m_irrDriver = pimpl->m_irrDevice->getVideoDriver();

    // Always visualize the mouse cursor
    pimpl->m_irrDevice->getCursorControl()->setVisible(true);

    // Add environment
    pimpl->m_environment.m_envNode       = pimpl->m_irrSmgr->addEmptySceneNode();
    pimpl->m_environment.m_rootFrameNode = addFrameAxes(pimpl->m_irrSmgr,pimpl->m_environment.m_envNode,visualizerOptions.rootFrameArrowsDimension);
    pimpl->m_environment.m_floorGridNode = addFloorGridNode(pimpl->m_irrSmgr,pimpl->m_environment.m_envNode);
    pimpl->m_environment.m_sceneManager = pimpl->m_irrSmgr;
    pimpl->m_environment.m_backgroundColor = irr::video::SColorf(visualizerOptions.backgroundColor.r,
                                                                 visualizerOptions.backgroundColor.g,
                                                                 visualizerOptions.backgroundColor.b,
                                                                 visualizerOptions.backgroundColor.a);

    pimpl->m_irrTextureBackgroundColor = irr::video::SColorf(textureOptions.backgroundColor.r,
                                                             textureOptions.backgroundColor.g,
                                                             textureOptions.backgroundColor.b,
                                                             textureOptions.backgroundColor.a);

    // Add default light (sun, directional light pointing backwards
    addVizLights(pimpl->m_irrSmgr);
    std::string sunName = "sun";
    pimpl->m_environment.addLight(sunName);
    ILight & sun = pimpl->m_environment.lightViz(sunName);
    sun.setType(DIRECTIONAL_LIGHT);
    sun.setDirection(iDynTree::Direction(0,0,-1));
    sun.setDiffuseColor(iDynTree::ColorViz(0.7,0.7,0.7,1.0));
    sun.setSpecularColor(iDynTree::ColorViz(0.1,0.1,0.1,1.0));
    sun.setAmbientColor(iDynTree::ColorViz(0.1,0.1,0.1,1.0));

    pimpl->m_camera.setIrrlichtCamera(addVizCamera(pimpl->m_irrSmgr));

    irr::scene::ISceneNodeAnimator* anm = new CameraAnimator(pimpl->m_irrDevice->getCursorControl());
    pimpl->m_camera.irrlichtCamera()->addAnimator(anm);
    anm->drop();

    pimpl->m_vectors.init(pimpl->m_irrSmgr);

    if (pimpl->m_irrDriver->queryFeature(irr::video::EVDF_RENDER_TO_TARGET)) //check if render to target is possible
    {
        pimpl->m_irrTexture = pimpl->m_irrDriver->addRenderTargetTexture(irr::core::dimension2d<irr::u32>(textureOptions.winWidth, textureOptions.winHeight), "RenderTexture");
    }

    pimpl->m_isInitialized = true;
    pimpl->lastFPS         = -1;

    return true;
#else
    IDYNTREE_UNUSED(visualizerOptions);
    IDYNTREE_UNUSED(textureOptions);
    reportError("Visualizer","init","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

bool Visualizer::init(const VisualizerOptions options)
{
    return init(options, options);
}

size_t Visualizer::getNrOfVisualizedModels()
{
#ifdef IDYNTREE_USES_IRRLICHT
    return pimpl->m_modelViz.size();
#else
    return 0;
#endif
}


std::string Visualizer::getModelInstanceName(size_t modelInstanceIndex)
{
    if( modelInstanceIndex >= getNrOfVisualizedModels() )
    {
        return "";
    }

#ifdef IDYNTREE_USES_IRRLICHT
    return pimpl->m_modelViz[modelInstanceIndex]->getInstanceName();
#else
    return "";
#endif

}

int Visualizer::getModelInstanceIndex(const std::string instanceName)
{
#ifdef IDYNTREE_USES_IRRLICHT
    for(size_t mdlInst=0; mdlInst < getNrOfVisualizedModels(); mdlInst++)
    {
        if( pimpl->m_modelViz[mdlInst]->getInstanceName() == instanceName )
        {
            return static_cast<int>(mdlInst);
        }
    }
#else
    IDYNTREE_UNUSED(instanceName);
#endif

    reportError("Visualizer","getModelInstanceIndex","Impossible to find model instance with the specified name");
    return -1;
}


bool Visualizer::addModel(const Model& model, const std::string& instanceName)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( !this->pimpl->m_isInitialized )
    {
        init();
    }

    if( !this->pimpl->m_isInitialized )
    {
        reportError("Visualizer","addModel","Error in initializing Irrlicht device");
        return false;
    }

    ModelVisualization * newModelViz = new ModelVisualization();

    if( !newModelViz->init(model,instanceName,pimpl->m_irrSmgr) )
    {
        delete newModelViz;
        return false;
    }

    this->pimpl->m_modelViz.push_back(newModelViz);

    return true;
#else
    IDYNTREE_UNUSED(model);
    IDYNTREE_UNUSED(instanceName);
    reportError("Visualizer","addModel","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

void Visualizer::draw()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( !pimpl->m_isInitialized )
    {
        reportError("Visualizer","draw","Impossible to run not initialized visualizer");
        return;
    }


    pimpl->m_irrDriver->beginScene(true,true, pimpl->m_environment.m_backgroundColor.toSColor());

    if (pimpl->m_irrTexture)
    {
        // set render target texture
        pimpl->m_irrDriver->setRenderTarget(pimpl->m_irrTexture, true, true, pimpl->m_irrTextureBackgroundColor.toSColor());

        auto textureDims = pimpl->m_irrTexture->getSize();

        pimpl->m_camera.irrlichtCamera()->setAspectRatio(textureDims.Width/ (float)textureDims.Height);

        // draw whole scene into render buffer
        pimpl->m_irrSmgr->drawAll();

        // set back old render target
        // The buffer might have been distorted, so clear it
        pimpl->m_irrDriver->setRenderTarget(0, true, true, pimpl->m_environment.m_backgroundColor.toSColor());
    }

    auto winDimensions = pimpl->m_irrDriver->getScreenSize();

    pimpl->m_camera.irrlichtCamera()->setAspectRatio(winDimensions.Width/ (float)winDimensions.Height);

    pimpl->m_irrSmgr->drawAll();

    pimpl->m_irrDriver->endScene();

    int fps = pimpl->m_irrDriver->getFPS();

    if (pimpl->lastFPS != fps)
    {
        irr::core::stringw str = L"iDynTree Visualizer [";
        str += pimpl->m_irrDriver->getName();
        str += "] FPS:";
        str += fps;
        str += " ";

        pimpl->m_irrDevice->setWindowCaption(str.c_str());
        pimpl->lastFPS = fps;
    }

#else
    reportError("Visualizer","init","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
#endif
}

bool Visualizer::drawToFile(const std::string filename)
{
    bool retValue = false;

    if( !pimpl->m_isInitialized )
    {
        reportError("Visualizer","drawToFile","Impossible to call drawToFile in a not initialized visualizer");
        return false;
    }

#ifdef IDYNTREE_USES_IRRLICHT
    // Method based on http://www.irrlicht3d.org/wiki/index.php?n=Main.TakingAScreenShot
    irr::video::IImage* const image = pimpl->m_irrDriver->createScreenShot();
    if (image) //should always be true, but you never know
    {
        //write screenshot to file
        if (!pimpl->m_irrDriver->writeImageToFile(image, filename.c_str()))
        {
            std::stringstream ss;
            ss << "Impossible to write image file to " << filename;
            reportError("Visualizer","drawToFile",ss.str().c_str());
            retValue = false;
        }

        //Don't forget to drop image since we don't need it anymore.
        image->drop();

        retValue = true;
    }
    else
    {
        reportError("Visualizer","drawToFile","Error in calling irr::video::IVideoDriver::createScreenShot method");
        return retValue = false;
    }
#else
    IDYNTREE_UNUSED(filename);
#endif

    return retValue;
}


IModelVisualization& Visualizer::modelViz(const std::string& instanceName)
{
    int idx = getModelInstanceIndex(instanceName);
    if( idx < 0 )
    {
        return this->pimpl->m_invalidModelViz;
    }

#ifdef IDYNTREE_USES_IRRLICHT
    return *(this->pimpl->m_modelViz[idx]);
#else
    return this->pimpl->m_invalidModelViz;
#endif
}

IModelVisualization& Visualizer::modelViz(size_t modelIdx)
{
#ifdef IDYNTREE_USES_IRRLICHT
    return *(this->pimpl->m_modelViz[modelIdx]);
#else
    return this->pimpl->m_invalidModelViz;
#endif
}

ICamera& Visualizer::camera()
{
    return pimpl->m_camera;
}

IEnvironment& Visualizer::enviroment()
{
    return pimpl->m_environment;
}

IVectorsVisualization &Visualizer::vectors()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( !this->pimpl->m_isInitialized )
    {
        init();
    }
    return this->pimpl->m_vectors;
#else
    return this->pimpl->m_invalidVectors;
#endif
}

bool Visualizer::run()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( !pimpl->m_isInitialized )
    {
        reportError("Visualizer","run","Impossible to run not initialized visualizer");
        return false;
    }

    return pimpl->m_irrDevice->run();
#else
    reportError("Visualizer","run","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

void Visualizer::close()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( !pimpl->m_isInitialized )
    {
        return;
    }

    pimpl->m_vectors.close();
    pimpl->m_environment.close();

    pimpl->m_irrDevice->closeDevice();
    pimpl->m_irrDevice->drop();
    pimpl->m_irrDevice = nullptr;
    pimpl->m_isInitialized = false;

    for(size_t mdl=0; mdl < pimpl->m_modelViz.size(); mdl++)
    {
        if( pimpl->m_modelViz[mdl] )
        {
            delete pimpl->m_modelViz[mdl];
            pimpl->m_modelViz[mdl] = nullptr;
        }
    }

    pimpl->m_modelViz.resize(0);

    return;
#endif
}

bool Visualizer::isWindowActive() const
{
#ifdef IDYNTREE_USES_IRRLICHT
    return pimpl->m_irrDevice->isWindowActive();
#else
    return false;
#endif
}

ColorViz Visualizer::getTexturePixelColor(unsigned int width, unsigned int height) const
{
    ColorViz pixelOut;
#ifdef IDYNTREE_USES_IRRLICHT
    irr::video::SColor pixelIrrlicht = irr::video::SColor(0, 0, 0, 0);

    if (!pimpl->m_irrTexture)
    {
        reportError("Visualizer","getTexturePixelColor","Cannot get pixel color. The video driver does not allow to render to a target.");
        return pixelOut;
    }

    auto textureDim = pimpl->m_irrTexture->getSize();

    if ( width >= textureDim.Width || height >= textureDim.Height)
    {
        std::stringstream ss;
        ss << "The requested pixel is out of bounds. Requested (" << width << ", " << height
           << "). Picture dimensions: (" << textureDim.Width  << ", " << textureDim.Height << ").";
        reportError("Visualizer", "getTexturePixelColor", ss.str().c_str());
        return pixelOut;
    }

    auto pitch = pimpl->m_irrTexture->getPitch();
    auto format = pimpl->m_irrTexture->getColorFormat();
    auto bytes = irr::video::IImage::getBitsPerPixelFromFormat(format) / 8;

    unsigned char* buffer = (unsigned char*) pimpl->m_irrTexture->lock(irr::video::E_TEXTURE_LOCK_MODE::ETLM_READ_ONLY);
    if (buffer)
    {
        pixelIrrlicht = irr::video::SColor(*(unsigned int*)(buffer + (height * pitch) + (width * bytes)));
        pimpl->m_irrTexture->unlock();
    }

    pixelOut.r = pixelIrrlicht.getRed();
    pixelOut.g = pixelIrrlicht.getGreen();
    pixelOut.b = pixelIrrlicht.getBlue();
    pixelOut.a = pixelIrrlicht.getAlpha();

    return pixelOut;
#else
    reportError("Visualizer","getTexturePixelColor","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return pixelOut;
#endif
}

bool Visualizer::getTexturePixels(std::vector<PixelViz> &pixels) const
{
#ifdef IDYNTREE_USES_IRRLICHT
    irr::video::SColor pixelIrrlicht = irr::video::SColor(0, 0, 0, 0);

    if (!pimpl->m_irrTexture)
    {
        reportError("Visualizer","getTexturePixels","Cannot get pixel color. The video driver does not allow to render to a target.");
        return false;
    }

    auto textureDim = pimpl->m_irrTexture->getSize();

    pixels.resize(textureDim.Width * textureDim.Height);

    auto pitch = pimpl->m_irrTexture->getPitch();
    auto format = pimpl->m_irrTexture->getColorFormat();
    auto bytes = irr::video::IImage::getBitsPerPixelFromFormat(format) / 8;

    unsigned char* buffer = (unsigned char*) pimpl->m_irrTexture->lock(irr::video::E_TEXTURE_LOCK_MODE::ETLM_READ_ONLY);
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

        pimpl->m_irrTexture->unlock();
    }

    return true;
#else
    reportError("Visualizer","getTexturePixels","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

}
