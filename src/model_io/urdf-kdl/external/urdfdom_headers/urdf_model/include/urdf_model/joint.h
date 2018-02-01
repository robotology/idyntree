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

#ifndef URDF_INTERFACE_JOINT_H
#define URDF_INTERFACE_JOINT_H

#include <string>
#include <vector>

#ifndef URDF_USE_PLAIN_POINTERS

#include <boost/shared_ptr.hpp>

#endif


#include "urdf_model/pose.h"


namespace urdf{

#ifdef URDF_USE_PLAIN_POINTERS
class JointDynamics;
class JointLimits;
class JointSafety;
class JointCalibration;
class JointMimic;

typedef double * DoublePtr;
typedef JointDynamics * JointDynamicsPtr;
typedef JointLimits   * JointLimitsPtr;
typedef JointSafety   * JointSafetyPtr;
typedef JointCalibration * JointCalibrationPtr;
typedef JointMimic    * JointMimicPtr;

inline void resetPtr(DoublePtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(DoublePtr & ptr, DoublePtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(JointDynamicsPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(JointDynamicsPtr & ptr, JointDynamicsPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(JointLimitsPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(JointLimitsPtr & ptr, JointLimitsPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(JointSafetyPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(JointSafetyPtr & ptr, JointSafetyPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(JointCalibrationPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(JointCalibrationPtr & ptr, JointCalibrationPtr plain_ptr) { ptr = plain_ptr; }
inline void resetPtr(JointMimicPtr & ptr) { if(ptr) { ptr=NULL; }  }
inline void resetPtr(JointMimicPtr & ptr, JointMimicPtr plain_ptr) { ptr = plain_ptr; }


#else

class JointDynamics;
class JointLimits;
class JointSafety;
class JointCalibration;
class JointMimic;

typedef boost::shared_ptr<double> DoublePtr;
typedef boost::shared_ptr<JointDynamics>  JointDynamicsPtr;
typedef boost::shared_ptr<JointLimits>    JointLimitsPtr;
typedef boost::shared_ptr<JointSafety>    JointSafetyPtr;
typedef boost::shared_ptr<JointCalibration> JointCalibrationPtr;
typedef boost::shared_ptr<JointMimic>     JointMimicPtr;
template<class PtrType> inline void resetPtr(PtrType & ptr) { ptr.reset(); }
template<class PtrType, class PlainType> inline void resetPtr(PtrType & ptr, PlainType * plain_ptr) { ptr.reset(plain_ptr); }


#endif


class Link;

class JointDynamics
{
public:
  JointDynamics() { this->clear(); };
  double damping;
  double friction;

  void clear()
  {
    damping = 0;
    friction = 0;
  };
};

class JointLimits
{
public:
  JointLimits() { this->clear(); };
  double lower;
  double upper;
  double effort;
  double velocity;

  void clear()
  {
    lower = 0;
    upper = 0;
    effort = 0;
    velocity = 0;
  };
};

/// \brief Parameters for Joint Safety Controllers
class JointSafety
{
public:
  /// clear variables on construction
  JointSafety() { this->clear(); };

  ///
  /// IMPORTANT:  The safety controller support is very much PR2 specific, not intended for generic usage.
  ///
  /// Basic safety controller operation is as follows
  ///
  /// current safety controllers will take effect on joints outside the position range below:
  ///
  /// position range: [JointSafety::soft_lower_limit  + JointLimits::velocity / JointSafety::k_position,
  ///                  JointSafety::soft_uppper_limit - JointLimits::velocity / JointSafety::k_position]
  ///
  /// if (joint_position is outside of the position range above)
  ///     velocity_limit_min = -JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_lower_limit)
  ///     velocity_limit_max =  JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_upper_limit)
  /// else
  ///     velocity_limit_min = -JointLimits::velocity
  ///     velocity_limit_max =  JointLimits::velocity
  ///
  /// velocity range: [velocity_limit_min + JointLimits::effort / JointSafety::k_velocity,
  ///                  velocity_limit_max - JointLimits::effort / JointSafety::k_velocity]
  ///
  /// if (joint_velocity is outside of the velocity range above)
  ///     effort_limit_min = -JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_min)
  ///     effort_limit_max =  JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_max)
  /// else
  ///     effort_limit_min = -JointLimits::effort
  ///     effort_limit_max =  JointLimits::effort
  ///
  /// Final effort command sent to the joint is saturated by [effort_limit_min,effort_limit_max]
  ///
  /// Please see wiki for more details: http://www.ros.org/wiki/pr2_controller_manager/safety_limits
  ///
  double soft_upper_limit;
  double soft_lower_limit;
  double k_position;
  double k_velocity;

  void clear()
  {
    soft_upper_limit = 0;
    soft_lower_limit = 0;
    k_position = 0;
    k_velocity = 0;
  };
};


class JointCalibration
{
public:
  JointCalibration() { this->clear(); };
  double reference_position;
  DoublePtr rising;
  DoublePtr falling;

  void clear()
  {
    reference_position = 0;
  };
};

class JointMimic
{
public:
  JointMimic() { this->clear(); };
  double offset;
  double multiplier;
  std::string joint_name;

  void clear()
  {
    offset = 0.0;
    multiplier = 0.0;
    joint_name.clear();
  };
};


class Joint
{
public:

  Joint() { this->clear(); };

  std::string name;
  enum
  {
    UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
  } type;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Vector3 axis;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;

  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;
  /// transform from Parent Link frame to Joint frame
  Pose  parent_to_joint_origin_transform;

  /// Joint Dynamics
  JointDynamicsPtr dynamics;

  /// Joint Limits
  JointLimitsPtr limits;

  /// Unsupported Hidden Feature
  JointSafetyPtr safety;

  /// Unsupported Hidden Feature
  JointCalibrationPtr calibration;

  /// Option to Mimic another Joint
  JointMimicPtr mimic;

  void clear()
  {
    this->axis.clear();
    this->child_link_name.clear();
    this->parent_link_name.clear();
    this->parent_to_joint_origin_transform.clear();

    resetPtr(this->dynamics);
    resetPtr(this->limits);
    resetPtr(this->safety);
    resetPtr(this->calibration);
    resetPtr(this->mimic);

    this->type = UNKNOWN;
  };
};

}

#endif
