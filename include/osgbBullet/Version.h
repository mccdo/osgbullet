/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009 by Kenneth Mark Bryden
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

#ifndef __OSGBBULLET_VERSION_H__
#define __OSGBBULLET_VERSION_H__ 1

#include "osgbBullet/Export.h"
#include <string>


namespace osgbBullet {


#define OSGBBULLET_MAJOR_VERSION 1
#define OSGBBULLET_MINOR_VERSION 0
#define OSGBBULLET_SUB_VERSION 0

// C preprocessor integrated version number.
// The form is Mmmss, where:
//   M is the major version
//   mm is the minor version (zero-padded)
//   ss is the sub version (zero padded)
// Use this in version-specific code, for example:
//   #if( OSGBBULLET_VERSION < 10500 )
//      ... code specific to releases before v1.05
//   #endif
#define OSGBBULLET_VERSION ( \
        ( OSGBBULLET_MAJOR_VERSION * 10000 ) + \
        ( OSGBBULLET_MINOR_VERSION * 100 ) + \
          OSGBBULLET_SUB_VERSION )

// Returns OSGBBULLET_VERSION.
unsigned int OSGBBULLET_EXPORT getVersionNumber();

// Pretty string.
std::string OSGBBULLET_EXPORT getVersionString();


// namespace osgbBullet
}

// __OSGBBULLET_VERSION_H__
#endif
