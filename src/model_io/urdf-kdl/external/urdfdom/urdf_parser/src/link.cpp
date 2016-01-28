/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */


#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <tinyxml.h>
#include "urdf_parser/outputdecl.h"
#include "urdf_model/boost_replacements.h"

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseMaterial(Material &material, TiXmlElement *config, bool only_name_is_ok)
{
  bool has_rgb = false;
  bool has_filename = false;

  material.clear();

  if (!config->Attribute("name"))
  {
    logError("Material must contain a name attribute");
    return false;
  }

  material.name = config->Attribute("name");

  // texture
  TiXmlElement *t = config->FirstChildElement("texture");
  if (t)
  {
    if (t->Attribute("filename"))
    {
      material.texture_filename = t->Attribute("filename");
      has_filename = true;
    }
  }

  // color
  TiXmlElement *c = config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba")) {

      try {
        material.color.init(c->Attribute("rgba"));
        has_rgb = true;
      }
      catch (ParseError &e) {
        material.color.clear();
        logError(std::string("Material [" + material.name + "] has malformed color rgba values: " + e.what()).c_str());
      }
    }
  }

  if (!has_rgb && !has_filename) {
    if (!only_name_is_ok) // no need for an error if only name is ok
    {
      if (!has_rgb) logError(std::string("Material ["+material.name+"] color has no rgba").c_str());
      if (!has_filename) logError(std::string("Material ["+material.name+"] not defined in file").c_str());
    }
    return false;
  }
  return true;
}


bool parseSphere(Sphere &s, TiXmlElement *c)
{
  s.clear();

  s.type = Geometry::SPHERE;
  if (!c->Attribute("radius"))
  {
    logError("Sphere shape must have a radius attribute");
    return false;
  }

  if( !stringToDouble(c->Attribute("radius"),s.radius) )
  {
    std::stringstream stm;
    stm << "radius [" << c->Attribute("radius") << "] is not a valid float: ";
    logError(stm.str().c_str());
    return false;
  }

  return true;
}

bool parseBox(Box &b, TiXmlElement *c)
{
  b.clear();

  b.type = Geometry::BOX;
  if (!c->Attribute("size"))
  {
    logError("Box shape has no size attribute");
    return false;
  }
  try
  {
    b.dim.init(c->Attribute("size"));
  }
  catch (ParseError &e)
  {
    b.dim.clear();
    logError(e.what());
    return false;
  }
  return true;
}

bool parseCylinder(Cylinder &y, TiXmlElement *c)
{
  y.clear();

  y.type = Geometry::CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    logError("Cylinder shape must have both length and radius attributes");
    return false;
  }

  if( !stringToDouble(c->Attribute("length"),y.length) )
  {
    std::stringstream stm;
    stm << "length [" << c->Attribute("length") << "] is not a valid float";
    logError(stm.str().c_str());
    return false;
  }

  if( !stringToDouble(c->Attribute("radius"),y.radius))
  {
    std::stringstream stm;
    stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
    logError(stm.str().c_str());
    return false;
  }
  return true;
}


bool parseMesh(Mesh &m, TiXmlElement *c)
{
  m.clear();

  m.type = Geometry::MESH;
  if (!c->Attribute("filename")) {
    logError("Mesh must contain a filename attribute");
    return false;
  }

  m.filename = c->Attribute("filename");

  if (c->Attribute("scale")) {
    try {
      m.scale.init(c->Attribute("scale"));
    }
    catch (ParseError &e) {
      m.scale.clear();
      logError("Mesh scale was specified, but could not be parsed: %s", e.what());
      return false;
    }
  }
  else
  {
    m.scale.x = m.scale.y = m.scale.z = 1;
  }
  return true;
}

GeometryPtr parseGeometry(TiXmlElement *g)
{
  GeometryPtr geom;
  if (!g) return geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    logError("Geometry tag contains no child element.");
    return geom;
  }

  std::string type_name = shape->ValueStr();
  if (type_name == "sphere")
  {
    Sphere *s = new Sphere();
    resetPtr(geom,s);
    if (parseSphere(*s, shape))
      return geom;
  }
  else if (type_name == "box")
  {
    Box *b = new Box();
    resetPtr(geom,b);
    if (parseBox(*b, shape))
      return geom;
  }
  else if (type_name == "cylinder")
  {
    Cylinder *c = new Cylinder();
    resetPtr(geom,c);
    if (parseCylinder(*c, shape))
      return geom;
  }
  else if (type_name == "mesh")
  {
    Mesh *m = new Mesh();
    resetPtr(geom,m);
    if (parseMesh(*m, shape))
      return geom;
  }
  else
  {
    logError("Unknown geometry type '%s'", type_name.c_str());
    return geom;
  }

  return GeometryPtr();
}

bool parseInertial(Inertial &i, TiXmlElement *config)
{
  i.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    if (!parsePose(i.origin, o))
      return false;
  }

  TiXmlElement *mass_xml = config->FirstChildElement("mass");
  if (!mass_xml)
  {
    logError("Inertial element must have a mass element");
    return false;
  }
  if (!mass_xml->Attribute("value"))
  {
    logError("Inertial: mass element must have value attribute");
    return false;
  }

  if( !stringToDouble(mass_xml->Attribute("value"),i.mass) )
  {
    std::stringstream stm;
    stm << "Inertial: mass [" << mass_xml->Attribute("value")
        << "] is not a float";
    logError(stm.str().c_str());
    return false;
  }

  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    logError("Inertial element must have inertia element");
    return false;
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    logError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
    return false;
  }

  if( !stringToDouble(inertia_xml->Attribute("ixx"),i.ixx)
      || !stringToDouble(inertia_xml->Attribute("ixy"),i.ixy)
      || !stringToDouble(inertia_xml->Attribute("ixz"),i.ixz)
      || !stringToDouble(inertia_xml->Attribute("iyy"),i.iyy)
      || !stringToDouble(inertia_xml->Attribute("iyz"),i.iyz)
      || !stringToDouble(inertia_xml->Attribute("izz"),i.izz) )

  {
    std::stringstream stm;
    stm << "Inertial: one of the inertia elements is not a valid double:"
        << " ixx [" << inertia_xml->Attribute("ixx") << "]"
        << " ixy [" << inertia_xml->Attribute("ixy") << "]"
        << " ixz [" << inertia_xml->Attribute("ixz") << "]"
        << " iyy [" << inertia_xml->Attribute("iyy") << "]"
        << " iyz [" << inertia_xml->Attribute("iyz") << "]"
        << " izz [" << inertia_xml->Attribute("izz") << "]";
    logError(stm.str().c_str());
    return false;
  }
  return true;
}

bool parseVisual(Visual &vis, TiXmlElement *config)
{
  vis.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    if (!parsePose(vis.origin, o))
      return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  vis.geometry = parseGeometry(geom);
  if (!vis.geometry)
    return false;

  const char *name_char = config->Attribute("name");
  if (name_char)
    vis.name = name_char;

  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (mat) {
    // get material name
    if (!mat->Attribute("name")) {
      logError("Visual material must contain a name attribute");
      return false;
    }
    vis.material_name = mat->Attribute("name");

    // try to parse material element in place
    resetPtr(vis.material,new Material());
    if (!parseMaterial(*vis.material, mat, true))
    {
      logDebug("urdfdom: material has only name, actual material definition may be in the model");
    }
  }

  return true;
}

bool parseCollision(Collision &col, TiXmlElement* config)
{
  col.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    if (!parsePose(col.origin, o))
      return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  col.geometry = parseGeometry(geom);
  if (!col.geometry)
    return false;

  const char *name_char = config->Attribute("name");
  if (name_char)
    col.name = name_char;

  return true;
}

bool parseLink(Link &link, TiXmlElement* config)
{

  link.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    logError("No name given for the link.");
    return false;
  }
  link.name = std::string(name_char);

  // Inertial (optional)
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    resetPtr(link.inertial,new Inertial());
    if (!parseInertial(*link.inertial, i))
    {
      logError("Could not parse inertial element for Link [%s]", link.name.c_str());
      return false;
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {

    VisualPtr vis;
    resetPtr(vis,new Visual());
    if (parseVisual(*vis, vis_xml))
    {
      link.visual_array.push_back(vis);
    }
    else
    {
      resetPtr(vis);
      logError("Could not parse visual element for Link [%s]", link.name.c_str());
      return false;
    }
  }

  // Visual (optional)
  // Assign the first visual to the .visual ptr, if it exists
  if (!link.visual_array.empty())
    link.visual = link.visual_array[0];

  // Multiple Collisions (optional)
  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
  {
    CollisionPtr col;
    resetPtr(col,new Collision());
    if (parseCollision(*col, col_xml))
    {
      link.collision_array.push_back(col);
    }
    else
    {
      resetPtr(col);
      logError("Could not parse collision element for Link [%s]",  link.name.c_str());
      return false;
    }
  }

  // Collision (optional)
  // Assign the first collision to the .collision ptr, if it exists
  if (!link.collision_array.empty())
    link.collision = link.collision_array[0];
}

/* exports */
bool exportPose(Pose &pose, TiXmlElement* xml);

bool exportMaterial(Material &material, TiXmlElement *xml)
{
  TiXmlElement *material_xml = new TiXmlElement("material");
  material_xml->SetAttribute("name", material.name);

  TiXmlElement* texture = new TiXmlElement("texture");
  if (!material.texture_filename.empty())
    texture->SetAttribute("filename", material.texture_filename);
  material_xml->LinkEndChild(texture);

  TiXmlElement* color = new TiXmlElement("color");
  color->SetAttribute("rgba", urdf_export_helpers::values2str(material.color));
  material_xml->LinkEndChild(color);
  xml->LinkEndChild(material_xml);
  return true;
}

bool exportSphere(Sphere &s, TiXmlElement *xml)
{
  // e.g. add <sphere radius="1"/>
  TiXmlElement *sphere_xml = new TiXmlElement("sphere");
  sphere_xml->SetAttribute("radius", urdf_export_helpers::values2str(s.radius));
  xml->LinkEndChild(sphere_xml);
  return true;
}

bool exportBox(Box &b, TiXmlElement *xml)
{
  // e.g. add <box size="1 1 1"/>
  TiXmlElement *box_xml = new TiXmlElement("box");
  box_xml->SetAttribute("size", urdf_export_helpers::values2str(b.dim));
  xml->LinkEndChild(box_xml);
  return true;
}

bool exportCylinder(Cylinder &y, TiXmlElement *xml)
{
  // e.g. add <cylinder radius="1"/>
  TiXmlElement *cylinder_xml = new TiXmlElement("cylinder");
  cylinder_xml->SetAttribute("radius", urdf_export_helpers::values2str(y.radius));
  cylinder_xml->SetAttribute("length", urdf_export_helpers::values2str(y.length));
  xml->LinkEndChild(cylinder_xml);
  return true;
}

bool exportMesh(Mesh &m, TiXmlElement *xml)
{
  // e.g. add <mesh filename="my_file" scale="1 1 1"/>
  TiXmlElement *mesh_xml = new TiXmlElement("mesh");
  if (!m.filename.empty())
    mesh_xml->SetAttribute("filename", m.filename);
  mesh_xml->SetAttribute("scale", urdf_export_helpers::values2str(m.scale));
  xml->LinkEndChild(mesh_xml);
  return true;
}

bool exportGeometry(GeometryPtr &geom, TiXmlElement *xml)
{
  TiXmlElement *geometry_xml = new TiXmlElement("geometry");
  if( geom->type == Geometry::SPHERE )
  {
    exportSphere(*((Sphere*)toPlainGeometryPtr(geom)), geometry_xml);
  }
  else if ( geom->type == Geometry::BOX )
  {
    exportBox(*((Box*)toPlainGeometryPtr(geom)), geometry_xml);
  }
  else if (geom->type == Geometry::CYLINDER)
  {
    exportCylinder(*((Cylinder*)toPlainGeometryPtr(geom)), geometry_xml);
  }
  else if (geom->type == Geometry::MESH)
  {
    exportMesh(*((Mesh*)toPlainGeometryPtr(geom)), geometry_xml);
  }
  else
  {
    logError("geometry not specified, I'll make one up for you!");
    Sphere *s = new Sphere();
    s->radius = 0.03;
    resetPtr(geom,s);
    exportSphere(*s, geometry_xml);
  }

  xml->LinkEndChild(geometry_xml);
  return true;
}

bool exportInertial(Inertial &i, TiXmlElement *xml)
{
  // adds <inertial>
  //        <mass value="1"/>
  //        <pose xyz="0 0 0" rpy="0 0 0"/>
  //        <inertia ixx="1" ixy="0" />
  //      </inertial>
  TiXmlElement *inertial_xml = new TiXmlElement("inertial");

  TiXmlElement *mass_xml = new TiXmlElement("mass");
  mass_xml->SetAttribute("value", urdf_export_helpers::values2str(i.mass));
  inertial_xml->LinkEndChild(mass_xml);

  exportPose(i.origin, inertial_xml);

  TiXmlElement *inertia_xml = new TiXmlElement("inertia");
  inertia_xml->SetAttribute("ixx", urdf_export_helpers::values2str(i.ixx));
  inertia_xml->SetAttribute("ixy", urdf_export_helpers::values2str(i.ixy));
  inertia_xml->SetAttribute("ixz", urdf_export_helpers::values2str(i.ixz));
  inertia_xml->SetAttribute("iyy", urdf_export_helpers::values2str(i.iyy));
  inertia_xml->SetAttribute("iyz", urdf_export_helpers::values2str(i.iyz));
  inertia_xml->SetAttribute("izz", urdf_export_helpers::values2str(i.izz));
  inertial_xml->LinkEndChild(inertia_xml);

  xml->LinkEndChild(inertial_xml);

  return true;
}

bool exportVisual(Visual &vis, TiXmlElement *xml)
{
  // <visual group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </visual>
  TiXmlElement * visual_xml = new TiXmlElement("visual");

  exportPose(vis.origin, visual_xml);

  exportGeometry(vis.geometry, visual_xml);

  if (vis.material)
    exportMaterial(*vis.material, visual_xml);

  xml->LinkEndChild(visual_xml);

  return true;
}

bool exportCollision(Collision &col, TiXmlElement* xml)
{
  // <collision group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </collision>
  TiXmlElement * collision_xml = new TiXmlElement("collision");

  exportPose(col.origin, collision_xml);

  exportGeometry(col.geometry, collision_xml);

  xml->LinkEndChild(collision_xml);

  return true;
}

bool exportLink(Link &link, TiXmlElement* xml)
{
  TiXmlElement * link_xml = new TiXmlElement("link");
  link_xml->SetAttribute("name", link.name);

  if (link.inertial)
    exportInertial(*link.inertial, link_xml);
  for (std::size_t i = 0 ; i < link.visual_array.size() ; ++i)
    exportVisual(*link.visual_array[i], link_xml);
  for (std::size_t i = 0 ; i < link.collision_array.size() ; ++i)
    exportCollision(*link.collision_array[i], link_xml);

  xml->LinkEndChild(link_xml);

  return true;
}

}
