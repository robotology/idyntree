// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Light.h"
#include "IrrlichtUtils.h"

namespace iDynTree
{

irr::video::E_LIGHT_TYPE idyntree2irrlicht(iDynTree::LightType type)
{
    switch (type)
    {
        case POINT_LIGHT:
            return irr::video::ELT_POINT;
        case DIRECTIONAL_LIGHT:
            return irr::video::ELT_DIRECTIONAL;
        default:
            return irr::video::ELT_COUNT;
    };
}

iDynTree::LightType irrlicht2idyntree(irr::video::E_LIGHT_TYPE type)
{
    switch (type)
    {
        case irr::video::ELT_POINT:
            return POINT_LIGHT;
        case irr::video::ELT_DIRECTIONAL:
            return POINT_LIGHT;
        default:
            return POINT_LIGHT;
    };
}

Light::Light(): m_lightSceneNode(0)
{
}

Light::~Light()
{
    removeLight();
}

void Light::addLight(const std::string _name, irr::scene::ILightSceneNode* _lightSceneNode)
{
    if( m_lightSceneNode )
    {
        removeLight();
    }

    m_name = _name;
    m_lightSceneNode = _lightSceneNode;
}

void Light::removeLight()
{
    if( m_lightSceneNode )
    {
        m_lightSceneNode->remove();
        m_lightSceneNode = 0;
    }
}

const std::string& Light::getName() const
{
    return m_name;
}

void Light::setType(const LightType type)
{
    if(m_lightSceneNode)
    {
        m_lightSceneNode->setLightType(idyntree2irrlicht(type));
    }
    else
    {
        reportError("Light","setType","Invalid light node");
    }
}

LightType Light::getType()
{
    if(m_lightSceneNode)
    {
        return irrlicht2idyntree(m_lightSceneNode->getLightType());
    }

    reportError("Light","getType","Invalid light node");
    return POINT_LIGHT;
}

void Light::setPosition(const Position& lightPos)
{
    if(m_lightSceneNode)
    {
        m_lightSceneNode->setPosition(idyntree2irr_pos(lightPos));
    }
    else
    {
        reportError("Light","setPosition","Invalid light node");
    }
}

Position Light::getPosition()
{
    if(m_lightSceneNode)
    {
        return irr2idyntree_pos(m_lightSceneNode->getPosition());
    }

    reportError("Light","getPosition","Invalid light node");
    return iDynTree::Position::Zero();
}

void Light::setDirection(const Direction& lightDirection)
{
    // From Irrlicht docs:
    // "If the light type is directional or
    // spot, the direction of the light source is defined by the rotation of the scene
    // node (assuming (0,0,1) as the local direction of the light)."
    // We then need to encode the direction in a world_R_light rotation
    Rotation world_R_light = RotationWithPrescribedZColumn(lightDirection);

    if(m_lightSceneNode)
    {
        m_lightSceneNode->setRotation(idyntree2irr_rot(world_R_light));
    }
    else
    {
        reportError("Light","setDirection","Invalid light node");
    }
}

Direction Light::getDirection()
{
    // From Irrlicht docs:
    // "If the light type is directional or
    // spot, the direction of the light source is defined by the rotation of the scene
    // node (assuming (0,0,1) as the local direction of the light)."
    // We then need to extract the direction from the light rotation,
    // i.e. by computing world_R_light * (0, 0, 1)

    if(m_lightSceneNode)
    {
        Rotation world_R_light = irr2idyntree_rot(m_lightSceneNode->getRotation());
        return world_R_light*Direction(0,0,1);
    }

    reportError("Light","getDirection","Invalid light node");
    return Direction::Default();
}

void Light::setAmbientColor(const ColorViz& ambientColor)
{
    if(m_lightSceneNode)
    {
        m_lightSceneNode->getLightData().AmbientColor = idyntree2irrlicht(ambientColor);
    }
    else
    {
        reportError("Light","setAmbientColor","Invalid light node");
    }
}

ColorViz Light::getAmbientColor()
{
    if(m_lightSceneNode)
    {
        return irrlicht2idyntree(m_lightSceneNode->getLightData().AmbientColor);
    }

    reportError("Light","getAmbientColor","Invalid light node");
    return ColorViz();
}

void Light::setDiffuseColor(const ColorViz& ambientColor)
{
    if(m_lightSceneNode)
    {
        m_lightSceneNode->getLightData().DiffuseColor = idyntree2irrlicht(ambientColor);
    }
    else
    {
        reportError("Light","setDiffuseColor","Invalid light node");
    }
}

ColorViz Light::getDiffuseColor()
{
    if(m_lightSceneNode)
    {
        return irrlicht2idyntree(m_lightSceneNode->getLightData().DiffuseColor);
    }

    reportError("Light","getDiffuseColor","Invalid light node");
    return ColorViz();
}

void Light::setSpecularColor(const ColorViz& ambientColor)
{
    if(m_lightSceneNode)
    {
        m_lightSceneNode->getLightData().SpecularColor = idyntree2irrlicht(ambientColor);
    }
    else
    {
        reportError("Light","setSpecularColor","Invalid light node");
    }
}

ColorViz Light::getSpecularColor()
{
    if(m_lightSceneNode)
    {
        return irrlicht2idyntree(m_lightSceneNode->getLightData().SpecularColor);
    }

    reportError("Light","getSpecularColor","Invalid light node");
    return ColorViz();
}

}
