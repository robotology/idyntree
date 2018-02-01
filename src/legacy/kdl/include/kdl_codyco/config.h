/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
