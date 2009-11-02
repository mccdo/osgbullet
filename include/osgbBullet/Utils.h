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

#ifndef __OSGBBULLET_UTILS_H__
#define __OSGBBULLET_UTILS_H__

#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec4>

#include <LinearMath/btTransform.h>

#include <osgbBullet/Export.h>


namespace osgbBullet {

OSGBBULLET_EXPORT osg::Matrix asOsgMatrix( const btTransform& t );
OSGBBULLET_EXPORT btTransform asBtTransform( const osg::Matrix& m );

OSGBBULLET_EXPORT osg::Matrix asOsgMatrix( const btMatrix3x3& m );
OSGBBULLET_EXPORT btMatrix3x3 asBtMatrix3x3( const osg::Matrix& m );

OSGBBULLET_EXPORT osg::Vec3 asOsgVec3( const btVector3& v );
OSGBBULLET_EXPORT btVector3 asBtVector3( const osg::Vec3& v );

OSGBBULLET_EXPORT osg::Vec4 asOsgVec4( const btVector3& v, const double w );
OSGBBULLET_EXPORT osg::Vec4 asOsgVec4( const btVector4& v );
OSGBBULLET_EXPORT btVector4 asBtVector4( const osg::Vec4& v );

} // end namespace osgbBullet

#endif // __OSGBBULLET_UTILS_H__
