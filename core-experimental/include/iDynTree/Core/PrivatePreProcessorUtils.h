/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_PRIVATE_PREPROCESSOR_UTILS_H
#define IDYNTREE_PRIVATE_PREPROCESSOR_UTILS_H

#ifdef __PRETTY_FUNCTION__
#define IDYNTREE_PRETTY_FUNCTION __PRETTY_FUNCTION__
#else 
#define IDYNTREE_PRETTY_FUNCTION __FUNCTION__
#endif

#endif /* IDYNTREE_PRIVATE_PREPROCESSOR_UTILS_H */
