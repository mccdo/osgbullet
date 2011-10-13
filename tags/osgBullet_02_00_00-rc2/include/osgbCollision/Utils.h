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

#ifndef __OSGBCOLLISION_UTILS_H__
#define __OSGBCOLLISION_UTILS_H__ 1


#include <osgbCollision/Export.h>

#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec4>

#include <LinearMath/btTransform.h>


namespace osgbCollision
{


/** \defgroup conversionutils Vector and Matrix Data Conversion Utilities
\brief Convenience functions for converting between OSG's and Bullet's vector and matrix classes.

*/
/**@{*/

OSGBCOLLISION_EXPORT osg::Matrix asOsgMatrix( const btTransform& t );
OSGBCOLLISION_EXPORT btTransform asBtTransform( const osg::Matrix& m );

OSGBCOLLISION_EXPORT osg::Matrix asOsgMatrix( const btMatrix3x3& m );
OSGBCOLLISION_EXPORT btMatrix3x3 asBtMatrix3x3( const osg::Matrix& m );

OSGBCOLLISION_EXPORT osg::Vec3 asOsgVec3( const btVector3& v );
OSGBCOLLISION_EXPORT btVector3 asBtVector3( const osg::Vec3& v );

OSGBCOLLISION_EXPORT osg::Vec4 asOsgVec4( const btVector3& v, const double w );
OSGBCOLLISION_EXPORT osg::Vec4 asOsgVec4( const btVector4& v );
OSGBCOLLISION_EXPORT btVector4 asBtVector4( const osg::Vec4& v );

/**@}*/


// osgbCollision
}


// __OSGBCOLLISION_UTILS_H__
#endif
