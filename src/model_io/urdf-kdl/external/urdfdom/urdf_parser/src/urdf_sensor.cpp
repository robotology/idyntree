/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: John Hsu */


#include <urdf_sensor/sensor.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <tinyxml.h>
#include "urdf_parser/outputdecl.h"

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseCamera(Camera &camera, TiXmlElement* config)
{
  camera.clear();
  camera.type = VisualSensor::CAMERA;

  TiXmlElement *image = config->FirstChildElement("image");
  if (image)
  {
    const char* width_char = image->Attribute("width");
    if (width_char)
    {
      if( !stringToUnsignedInt(width_char,camera.width) )
      {
        logError("Camera image width [%s] is not a valid int", width_char);
        return false;
      }
    }
    else
    {
      logError("Camera sensor needs an image width attribute");
      return false;
    }

    const char* height_char = image->Attribute("height");
    if (height_char)
    {
      if( !stringToInt(height_char,camera.height) )
      {
        logError("Camera image height [%s] is not a valid int", height_char);
        return false;
      }
    }
    else
    {
      logError("Camera sensor needs an image height attribute");
      return false;
    }

    const char* format_char = image->Attribute("format");
    if (format_char)
      camera.format = std::string(format_char);
    else
    {
      logError("Camera sensor needs an image format attribute");
      return false;
    }

    const char* hfov_char = image->Attribute("hfov");
    if (hfov_char)
    {
      if( !stringToDouble(hfov_char,camera.hvof) )
      {
        logError("Camera image hfov [%s] is not a valid float", hfov_char);
        return false;
      }
    }
    else
    {
      logError("Camera sensor needs an image hfov attribute");
      return false;
    }

    const char* near_char = image->Attribute("near");
    if (near_char)
    {
      if( !stringToDouble(near_char,camera.near) )
      {
        logError("Camera image near [%s] is not a valid float", near_char);
        return false;
      }
    }
    else
    {
      logError("Camera sensor needs an image near attribute");
      return false;
    }

    const char* far_char = image->Attribute("far");
    if (far_char)
    {
      if( !stringToDouble(far_char,camera.far) )
      {
        logError("Camera image far [%s] is not a valid float", far_char);
        return false;
      }
    }
    else
    {
      logError("Camera sensor needs an image far attribute");
      return false;
    }

  }
  else
  {
    logError("Camera sensor has no <image> element");
    return false;
  }
  return true;
}

bool parseRay(Ray &ray, TiXmlElement* config)
{
  ray.clear();
  ray.type = VisualSensor::RAY;

  TiXmlElement *horizontal = config->FirstChildElement("horizontal");
  if (horizontal)
  {
    const char* samples_char = horizontal->Attribute("samples");
    if (samples_char)
    {
      if( !stringToInt(samples_char,camera.horizontal_samples) )
      {
        logError("Ray horizontal samples [%s] is not a valid float", samples_char);
        return false;
      }
    }

    const char* resolution_char = horizontal->Attribute("resolution");
    if (resolution_char)
    {
      if( !stringToDouble(resoulution_char,camera.horizontal_resolution) )
      {
        logError("Ray horizontal resolution [%s] is not a valid float: %s", resolution_char, e.what());
        return false;
      }
    }

    const char* min_angle_char = horizontal->Attribute("min_angle");
    if (min_angle_char)
    {
      if( !stringToDouble(min_angle_char,ray.horizontal_min_angle) )
      {
        logError("Ray horizontal min_angle [%s] is not a valid float", min_angle_char);
        return false;
      }
    }

    const char* max_angle_char = horizontal->Attribute("max_angle");
    if (max_angle_char)
    {
      if( !stringToDouble(max_angle_char,ray.horizontal_max_angle) )
      {
        logError("Ray horizontal max_angle [%s] is not a valid float", max_angle_char);
        return false;
      }
    }
  }

  TiXmlElement *vertical = config->FirstChildElement("vertical");
  if (vertical)
  {
    const char* samples_char = vertical->Attribute("samples");
    if (samples_char)
    {
      try
      {
        ray.vertical_samples = boost::lexical_cast<unsigned int>(samples_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        logError("Ray vertical samples [%s] is not a valid float: %s", samples_char, e.what());
        return false;
      }
    }

    const char* resolution_char = vertical->Attribute("resolution");
    if (resolution_char)
    {
      try
      {
        ray.vertical_resolution = boost::lexical_cast<double>(resolution_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        logError("Ray vertical resolution [%s] is not a valid float: %s", resolution_char, e.what());
        return false;
      }
    }

    const char* min_angle_char = vertical->Attribute("min_angle");
    if (min_angle_char)
    {
      try
      {
        ray.vertical_min_angle = boost::lexical_cast<double>(min_angle_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        logError("Ray vertical min_angle [%s] is not a valid float: %s", min_angle_char, e.what());
        return false;
      }
    }

    const char* max_angle_char = vertical->Attribute("max_angle");
    if (max_angle_char)
    {
      try
      {
        ray.vertical_max_angle = boost::lexical_cast<double>(max_angle_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        logError("Ray vertical max_angle [%s] is not a valid float: %s", max_angle_char, e.what());
        return false;
      }
    }
  }
}

boost::shared_ptr<VisualSensor> parseVisualSensor(TiXmlElement *g)
{
  boost::shared_ptr<VisualSensor> visual_sensor;

  // get sensor type
  TiXmlElement *sensor_xml;
  if (g->FirstChildElement("camera"))
  {
    Camera *camera = new Camera();
    visual_sensor.reset(camera);
    sensor_xml = g->FirstChildElement("camera");
    if (!parseCamera(*camera, sensor_xml))
      visual_sensor.reset();
  }
  else if (g->FirstChildElement("ray"))
  {
    Ray *ray = new Ray();
    visual_sensor.reset(ray);
    sensor_xml = g->FirstChildElement("ray");
    if (!parseRay(*ray, sensor_xml))
      visual_sensor.reset();
  }
  else
  {
    logError("No know sensor types [camera|ray] defined in <sensor> block");
  }
  return visual_sensor;
}


bool parseSensor(Sensor &sensor, TiXmlElement* config)
{
  sensor.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    logError("No name given for the sensor.");
    return false;
  }
  sensor.name = std::string(name_char);

  // parse parent_link_name
  const char *parent_link_name_char = config->Attribute("parent_link_name");
  if (!parent_link_name_char)
  {
    logError("No parent_link_name given for the sensor.");
    return false;
  }
  sensor.parent_link_name = std::string(parent_link_name_char);

  // parse origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    if (!parsePose(sensor.origin, o))
      return false;
  }

  // parse sensor
  sensor.sensor = parseVisualSensor(config);
  return true;
}


}


