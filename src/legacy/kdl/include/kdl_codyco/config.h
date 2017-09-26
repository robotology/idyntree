/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 

#ifndef KDL_CODYCO_CONFIG_H
#define KDL_CODYCO_CONFIG_H

#ifdef __DEPRECATED
  #warning <config.h> is deprecated.
#endif

#ifndef KDLCodycoOrocosKDLMinVersion
#define KDLCodycoOrocosKDLMinVersion(Major, Minor, Patch) \
OROCOS_KDL_VERSION_MAJOR > Major \
|| (Major == OROCOS_KDL_VERSION_MAJOR && OROCOS_KDL_VERSION_MINOR > Minor)\
|| (Major == OROCOS_KDL_VERSION_MAJOR && Minor == OROCOS_KDL_VERSION_MINOR && OROCOS_KDL_VERSION_PATCH >= Patch)
#endif

#if !(KDLCodycoOrocosKDLMinVersion(1, 2, 3))
#define GetTreeElementChildren(tree_element) (tree_element).children
#define GetTreeElementParent(tree_element) (tree_element).parent
#define GetTreeElementQNr(tree_element) (tree_element).q_nr
#define GetTreeElementSegment(tree_element) (tree_element).segment
#endif

#endif
