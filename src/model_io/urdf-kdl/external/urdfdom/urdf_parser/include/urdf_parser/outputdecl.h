/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef URDFDOM_OUTPUTDECL_H
#define URDFDOM_OUTPUTDECL_H

#ifndef URDFDOM_DO_NOT_USE_CONSOLEBRIDGE
// If not additional option is specified, use the control_bridge functions
# include <console_bridge/console.h>

#else /* URDFDOM_DO_NOT_USE_CONSOLEBRIDGE */
// If the user do not want to depend on console_bridge, define the output
// function using standard C functions
#include <cstdio>

# define logError(fmt, ...)  fprintf(stderr, fmt, ##__VA_ARGS__)
# define logWarn(fmt, ...)   fprintf(stderr, fmt, ##__VA_ARGS__)
# define logInform(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
# ifndef NDEBUG
# define logDebug(fmt, ...)  fprintf(stderr, fmt, ##__VA_ARGS__)
# else
# define logDebug(fmt, ...)
# endif

#endif /* URDFDOM_DO_NOT_USE_CONSOLEBRIDGE */

#endif /* URDFDOM_OUTPUTDECL_H */
