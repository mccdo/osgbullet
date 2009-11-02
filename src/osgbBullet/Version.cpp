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

#include "osgbBullet/Version.h"
#include <string>
#include <sstream>

namespace osgbBullet {


unsigned int
getVersionNumber()
{
    return( OSGBBULLET_VERSION );
}


static std::string s_osgbullet_version( "" );

std::string
getVersionString()
{
    if( s_osgbullet_version.empty() )
    {
        std::ostringstream oStr;
        oStr << std::string( "osgbBullet version " ) <<
            OSGBBULLET_MAJOR_VERSION << "." <<
            OSGBBULLET_MINOR_VERSION << "." <<
            OSGBBULLET_SUB_VERSION << " (" <<
            getVersionNumber() << ").";
        s_osgbullet_version = oStr.str();
    }
    return( s_osgbullet_version );
}


// namespace osgbBullet
}
