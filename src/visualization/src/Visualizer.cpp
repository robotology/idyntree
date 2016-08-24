/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Visualizer.h>

#ifdef IDYNTREE_USES_IRRLICHT
#include <irrlicht.h>
#include "IrrlichtUtils.h"
#endif

#include <cassert>

namespace iDynTree
{

struct Visualizer::VisualizerPimpl
{
    /**
     * True if init has been called.
     */
    bool m_isInitialized;

    /**
     * Collection of model visualization.
     */
    std::vector<ModelVisualization*> m_modelViz;

    /**
     * Invalid model visualization, useful to return in case of error.
     */
    ModelVisualization m_invalidModelViz;

    /**
     * Last FPS measured.
     */
    int lastFPS;

#ifdef IDYNTREE_USES_IRRLICHT
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
     * Camera used by the visualization.
     */
    irr::scene::ICameraSceneNode * m_irrCamera;
#endif

    VisualizerPimpl()
    {
        m_isInitialized = false;
        m_modelViz.resize(0);
        lastFPS = -1;

#ifdef IDYNTREE_USES_IRRLICHT
        m_irrDevice = 0;
        m_irrSmgr   = 0;
        m_irrDriver = 0;
        m_irrCamera = 0;
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

bool Visualizer::init(const VisualizerOptions options)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if( pimpl->m_isInitialized )
    {
        reportWarning("Visualizer","init","Visualier already initialized, call close() to close it to open it again.");
        return false;
    }

    irr::SIrrlichtCreationParameters irrDevParams;

    irrDevParams.DriverType = irr::video::EDT_OPENGL;

    if( options.verbose )
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

    // Add visualization elements
    addVizEnviroment(pimpl->m_irrSmgr);
    addVizLights(pimpl->m_irrSmgr);
    pimpl->m_irrCamera = addVizCamera(pimpl->m_irrSmgr);

    pimpl->m_isInitialized = true;
    pimpl->lastFPS         = -1;

    return true;
#else
    reportError("Visualizer","init","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
    return false;
#endif
}

size_t Visualizer::getNrOfVisualizedModels()
{
    return pimpl->m_modelViz.size();
}


std::string Visualizer::getModelInstanceName(size_t modelInstanceIndex)
{
    if( modelInstanceIndex >= getNrOfVisualizedModels() )
    {
        return "";
    }

    return pimpl->m_modelViz[modelInstanceIndex]->getInstanceName();
}

int Visualizer::getModelInstanceIndex(const std::string instanceName)
{
    for(size_t mdlInst=0; mdlInst < getNrOfVisualizedModels(); mdlInst++)
    {
        if( pimpl->m_modelViz[mdlInst]->getInstanceName() == instanceName )
        {
            return static_cast<int>(mdlInst);
        }
    }

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

    if( !newModelViz->init(model,instanceName,*this) )
    {
        delete newModelViz;
        return false;
    }

    this->pimpl->m_modelViz.push_back(newModelViz);

    return true;
#else
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

    pimpl->m_irrDriver->beginScene(true,true, irr::video::SColor(255,0,100,100));

    // Draw base plane
    for(int i=-10; i <= 10; i++ )
    {
        pimpl->m_irrDriver->draw3DLine(irr::core::vector3df(-10,i,0),
                                       irr::core::vector3df(10,i,0),
                                       irr::video::SColor(100,100,100,100));
        pimpl->m_irrDriver->draw3DLine(irr::core::vector3df(i,-10,0),
                                       irr::core::vector3df(i,10,0),
                                       irr::video::SColor(100,100,100,100));
    }
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

#endif

    return retValue;
}


ModelVisualization& Visualizer::modelViz(const std::string& instanceName)
{
    int idx = getModelInstanceIndex(instanceName);
    if( idx < 0 )
    {
        return this->pimpl->m_invalidModelViz;
    }

    return *(this->pimpl->m_modelViz[idx]);
}

ModelVisualization& Visualizer::modelViz(size_t modelIdx)
{
    return *(this->pimpl->m_modelViz[modelIdx]);
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
    reportError("Visualizer","init","Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without Irrlicht.");
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

    pimpl->m_irrDevice->closeDevice();
    pimpl->m_irrDevice->drop();
    pimpl->m_irrDevice = 0;
    pimpl->m_isInitialized = false;

    for(size_t mdl=0; mdl < pimpl->m_modelViz.size(); mdl++)
    {
        if( pimpl->m_modelViz[mdl] )
        {
            delete pimpl->m_modelViz[mdl];
            pimpl->m_modelViz[mdl] = 0;
        }
    }

    pimpl->m_modelViz.resize(0);

    return;
#endif
}

}
