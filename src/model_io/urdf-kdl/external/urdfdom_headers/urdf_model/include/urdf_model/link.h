/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Wim Meeussen */

#ifndef URDF_INTERFACE_LINK_H
#define URDF_INTERFACE_LINK_H

#include <string>
#include <vector>
#include <map>

#ifndef URDF_USE_PLAIN_POINTERS

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#endif

#include "joint.h"
#include "color.h"

namespace urdf{

#ifdef URDF_USE_PLAIN_POINTERS

class Visual;
class Geometry;
class Collision;
class Inertial;
class Material;
class Joint;
class Link;

typedef Visual *    VisualPtr;
typedef Geometry *  GeometryPtr;
typedef Collision * CollisionPtr;
typedef Inertial *  InertialPtr;
typedef Material *  MaterialPtr;
typedef Joint    *  JointPtr;
typedef const Joint * ConstJointPtr;
typedef Link     *  LinkPtr;
typedef Link     *  LinkWeakPtr;
typedef  const Link * ConstLinkPtr;

inline void resetPtr(VisualPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(VisualPtr & ptr, VisualPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(GeometryPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(GeometryPtr & ptr, GeometryPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(CollisionPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(CollisionPtr & ptr, CollisionPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(InertialPtr & ptr) { if(ptr) {  ptr=NULL; }  }
inline void resetPtr(InertialPtr & ptr, InertialPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(MaterialPtr & ptr) { if(ptr) {ptr=NULL; }  }
inline void resetPtr(MaterialPtr & ptr, MaterialPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(JointPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(JointPtr & ptr, JointPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(LinkPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(LinkPtr & ptr, LinkPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(ConstJointPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(ConstLinkPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline LinkPtr LinkWeakPtrToLinkPtr(LinkWeakPtr & ptr) { return (LinkPtr) ptr; }
inline LinkPtr LinkWeakPtrToLinkPtr(const LinkWeakPtr & ptr) { return (LinkPtr) ptr; }
inline Geometry* toPlainGeometryPtr(GeometryPtr & geom) { return geom; }

#else

typedef boost::shared_ptr<Visual> VisualPtr;
typedef boost::shared_ptr<Geometry> GeometryPtr;
typedef boost::shared_ptr<Collision> CollisionPtr;
typedef boost::shared_ptr<Inertial> InertialPtr;
typedef boost::shared_ptr<Material> MaterialPtr;
typedef boost::shared_ptr<Joint>    JointPtr;
typedef boost::shared_ptr<Link>     LinkPtr;
typedef boost::weak_ptr<Link>       LinkWeakPtr;
typedef boost::shared_ptr<const Link> ConstLinkPtr;
typedef boost::shared_ptr<const Joint> ConstJointPtr;
inline LinkPtr LinkWeakPtrToLinkPtr(LinkWeakPtr & ptr) { return ptr.lock(); }
inline Geometry* toPlainGeometryPtr(GeometryPtr & geom) { return geom.get(); }

#endif


class Geometry
{
public:
  enum {SPHERE, BOX, CYLINDER, MESH} type;

  virtual ~Geometry(void)
  {
  }
};

class Sphere : public Geometry
{
public:
  Sphere() { this->clear(); type = SPHERE; };
  double radius;

  void clear()
  {
    radius = 0;
  };
};

class Box : public Geometry
{
public:
  Box() { this->clear(); type = BOX; };
  Vector3 dim;

  void clear()
  {
    this->dim.clear();
  };
};

class Cylinder : public Geometry
{
public:
  Cylinder() { this->clear(); type = CYLINDER; };
  double length;
  double radius;

  void clear()
  {
    length = 0;
    radius = 0;
  };
};

class Mesh : public Geometry
{
public:
  Mesh() { this->clear(); type = MESH; };
  std::string filename;
  Vector3 scale;

  void clear()
  {
    filename.clear();
    // default scale
    scale.x = 1;
    scale.y = 1;
    scale.z = 1;
  };
};

class Material
{
public:
  Material() { this->clear(); };
  std::string name;
  std::string texture_filename;
  Color color;

  void clear()
  {
    color.clear();
    texture_filename.clear();
    name.clear();
  };
};

class Inertial
{
public:
  Inertial() { this->clear(); };
  Pose origin;
  double mass;
  double ixx,ixy,ixz,iyy,iyz,izz;

  void clear()
  {
    origin.clear();
    mass = 0;
    ixx = ixy = ixz = iyy = iyz = izz = 0;
  };
};

class Visual
{
public:
  Visual() { this->clear(); };
  Pose origin;
  GeometryPtr geometry;

  std::string material_name;
  MaterialPtr material;

  void clear()
  {
    origin.clear();
    material_name.clear();
    resetPtr(material);
    resetPtr(geometry);
    name.clear();
  };

  std::string name;
};

class Collision
{
public:
  Collision() { this->clear(); };
  Pose origin;
  GeometryPtr geometry;

  void clear()
  {
    origin.clear();
    resetPtr(geometry);
    name.clear();
  };

  std::string name;

};


class Link
{
public:
  Link() { this->clear(); };

  std::string name;

  /// inertial element
  InertialPtr inertial;

  /// visual element
  VisualPtr visual;

  /// collision element
  CollisionPtr collision;

  /// if more than one collision element is specified, all collision elements are placed in this array (the collision member points to the first element of the array)
  std::vector< CollisionPtr > collision_array;

  /// if more than one visual element is specified, all visual elements are placed in this array (the visual member points to the first element of the array)
  std::vector< VisualPtr > visual_array;

  /// Parent Joint element
  ///   explicitly stating "parent" because we want directional-ness for tree structure
  ///   every link can have one parent
  JointPtr parent_joint;

  std::vector<JointPtr > child_joints;
  std::vector<LinkPtr > child_links;

  LinkPtr getParent() const
  {return  LinkWeakPtrToLinkPtr(parent_link_);};

  void setParent(const LinkPtr &parent)
  { parent_link_ = parent; }

  void clear()
  {
    this->name.clear();
    resetPtr(this->inertial);
    resetPtr(this->visual);
    resetPtr(this->collision);
    resetPtr(this->parent_joint);
    this->child_joints.clear();
    this->child_links.clear();
    this->collision_array.clear();
    this->visual_array.clear();
  };

private:
  LinkWeakPtr parent_link_;

};




}

#endif
