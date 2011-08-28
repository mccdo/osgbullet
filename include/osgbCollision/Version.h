/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#ifndef __OSGBCOLLISION_VERSION_H__
#define __OSGBCOLLISION_VERSION_H__ 1

#include <osgbCollision/Export.h>
#include <string>


namespace osgbCollision {


// Please keep in sync with top-level CMakeLists.txt OSGBULLET_VERSION variable.
#define OSGBCOLLISION_MAJOR_VERSION 1
#define OSGBCOLLISION_MINOR_VERSION 1
#define OSGBCOLLISION_SUB_VERSION 81

// C preprocessor integrated version number.
// The form is Mmmss, where:
//   M is the major version
//   mm is the minor version (zero-padded)
//   ss is the sub version (zero padded)
// Use this in version-specific code, for example:
//   #if( OSGBCOLLISION_VERSION < 10500 )
//      ... code specific to releases before v1.05
//   #endif
#define OSGBCOLLISION_VERSION ( \
        ( OSGBCOLLISION_MAJOR_VERSION * 10000 ) + \
        ( OSGBCOLLISION_MINOR_VERSION * 100 ) + \
          OSGBCOLLISION_SUB_VERSION )

// Returns OSGBCOLLISION_VERSION.
unsigned int OSGBCOLLISION_EXPORT getVersionNumber();

// Pretty string.
std::string OSGBCOLLISION_EXPORT getVersionString();


// Backwards compatibility
#define OSGBBULLET_MAJOR_VERSION OSGBCOLLISION_MAJOR_VERSION
#define OSGBBULLET_MINOR_VERSION OSGBCOLLISION_MINOR_VERSION
#define OSGBBULLET_SUB_VERSION OSGBCOLLISION_SUB_VERSION
#define OSGBBULLET_VERSION OSGBCOLLISION_VERSION


// namespace osgbCollision
}


// __OSGBCOLLISION_VERSION_H__
#endif


/** \mainpage osgBullet Documentation

\section Introduction Introduction

TBD

\section Libraries Libraries

\subsection osgbCollision osgbCollision

Collision detection and collision shape support. Facilities for
creating Bullet collision shapes from OSG scene graphs, and vice
versa.

\subsection osgbDynamics osgbDynamics

Rigid body dynamics and constraints support.

\subsection osgdb_osgbDynamics osgdb_osgbDynamics

Dot OSG file support for classes and objects in the osgbDynamics library.

*/
