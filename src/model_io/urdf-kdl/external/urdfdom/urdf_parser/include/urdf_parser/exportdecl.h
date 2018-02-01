/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Thomas Moulard */

#ifndef URDFDOM_EXPORTDECL_H
# define URDFDOM_EXPORTDECL_H

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define URDFDOM_DLLIMPORT __declspec(dllimport)
#  define URDFDOM_DLLEXPORT __declspec(dllexport)
#  define URDFDOM_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define URDFDOM_DLLIMPORT __attribute__ ((visibility("default")))
#   define URDFDOM_DLLEXPORT __attribute__ ((visibility("default")))
#   define URDFDOM_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define URDFDOM_DLLIMPORT
#   define URDFDOM_DLLEXPORT
#   define URDFDOM_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef URDFDOM_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define URDFDOM_DLLAPI
#  define URDFDOM_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef console_bridge_EXPORTS
#   define URDFDOM_DLLAPI URDFDOM_DLLEXPORT
#  else
#   define URDFDOM_DLLAPI URDFDOM_DLLIMPORT
#  endif // URDFDOM_EXPORTS
#  define URDFDOM_LOCAL URDFDOM_DLLLOCAL
# endif // URDFDOM_STATIC
#endif //! URDFDOM_EXPORTDECL_H
