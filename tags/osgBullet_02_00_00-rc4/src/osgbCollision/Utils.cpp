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

#include <osgbCollision/Utils.h>
#include <osg/Matrix>
#include <LinearMath/btTransform.h>


using namespace osgbCollision;


// Convert a btTransform to an OSG Matrix
osg::Matrix
osgbCollision::asOsgMatrix( const btTransform& t )
{
    btScalar ogl[ 16 ];
    t.getOpenGLMatrix( ogl );
    osg::Matrix m( ogl );
    return m;
}

btTransform
osgbCollision::asBtTransform( const osg::Matrix& m )
{
    const osg::Matrix::value_type* oPtr = m.ptr();
    btScalar bPtr[ 16 ];
    int idx;
    for (idx=0; idx<16; idx++)
        bPtr[ idx ] = oPtr[ idx ];
    btTransform t;
    t.setFromOpenGLMatrix( bPtr );
    return t;
}


osg::Matrix
osgbCollision::asOsgMatrix( const btMatrix3x3& m )
{
    btScalar f[ 9 ];
    m.getOpenGLSubMatrix( f );
    return( osg::Matrix(
        f[0], f[1], f[2], 0.,
        f[3], f[4], f[5], 0.,
        f[6], f[7], f[8], 0.,
        0., 0., 0., 1. ) );
}

btMatrix3x3
osgbCollision::asBtMatrix3x3( const osg::Matrix& m )
{
    return( btMatrix3x3(
        m(0,0), m(0,1), m(0,2),
        m(1,0), m(1,1), m(1,2),
        m(2,0), m(2,1), m(2,2) ) );
}


osg::Vec3
osgbCollision::asOsgVec3( const btVector3& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}

btVector3
osgbCollision::asBtVector3( const osg::Vec3& v )
{
    return btVector3( v.x(), v.y(), v.z() );
}


osg::Vec4
osgbCollision::asOsgVec4( const btVector3& v, const double w )
{
    return osg::Vec4( v.x(), v.y(), v.z(), w );
}

osg::Vec4
osgbCollision::asOsgVec4( const btVector4& v )
{
    return osg::Vec4( v.x(), v.y(), v.z(), v.w() );
}

btVector4
osgbCollision::asBtVector4( const osg::Vec4& v )
{
    return btVector4( v.x(), v.y(), v.z(), v.w() );
}

