/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_PRIVATE_PREPROCESSOR_UTILS_H
#define IDYNTREE_PRIVATE_PREPROCESSOR_UTILS_H

#ifdef __PRETTY_FUNCTION__
#define IDYNTREE_PRETTY_FUNCTION __PRETTY_FUNCTION__
#else 
#define IDYNTREE_PRETTY_FUNCTION __FUNCTION__
#endif

#endif /* IDYNTREE_PRIVATE_PREPROCESSOR_UTILS_H */
