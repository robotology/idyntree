/**
 * Copyright  (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

/**
 * Header to mantain iDynTree parser compatible both with the official
 * urdfdom parser available at github.com/ros/urdfdom(_headers) and from
 * deb packages and with the internal fork of urdfdom in which boost has been
 * removed for the sake of an easy installation on Windows.
 */
#ifndef IDYNTREE_MODELIO_URDFCOMPATIBILITY_H
#define IDYNTREE_MODELIO_URDFCOMPATIBILITY_H

#ifndef IDYNTREE_USE_INTERNAL_URDFDOM

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace urdf
{

typedef boost::shared_ptr<Joint> JointPtr;
typedef boost::shared_ptr<Link>  LinkPtr;
typedef boost::shared_ptr<const Link>  ConstLinkPtr;
typedef boost::shared_ptr<Inertial> InertialPtr;
typedef std::vector<boost::shared_ptr<urdf::Link> > LinkVector;
typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfacePtr;
typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > JointPtrMap;

}

#else

#endif

#endif
