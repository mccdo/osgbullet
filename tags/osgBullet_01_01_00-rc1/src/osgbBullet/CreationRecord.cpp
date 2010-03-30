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

#include "osgbBullet/CreationRecord.h"

#include <osgbBullet/CollisionShapes.h>

#include <osg/Object>
#include <osg/Vec3>

#include <btBulletDynamicsCommon.h>

#include <string>


using namespace osgbBullet;


CreationRecord::CreationRecord()
  : _sceneGraph( NULL ),
    _version( 1 ),
    _com( 0., 0., 0. ),
    _comSet( false ),
    _scale( osg::Vec3( 1., 1., 1. ) ),
    _shapeType( BOX_SHAPE_PROXYTYPE ),
    _mass( 1.f ),
    _decimatorPercent( 1.f ),
    _decimatorMaxError( FLT_MAX ),
    _decimatorIgnoreBoundaries( true ),
    _simplifyPercent( 1.f ),
    _vertexAggMaxVerts( 0 ),
    _vertexAggMinCellSize( osg::Vec3( 0., 0., 0.) ),
    _reducerGroupThreshold( 360.f ),
    _reducerMaxEdgeError( 360.f ),
    _overall( true ),
    _nodeName( "" ),
    _axis( osgbBullet::Z )
{
}
CreationRecord::CreationRecord( const CreationRecord& rhs, osg::CopyOp copyop )
  : _sceneGraph( rhs._sceneGraph ),
    _version( rhs._version ),
    _com( rhs._com ),
    _comSet( rhs._comSet ),
    _scale( rhs._scale ),
    _shapeType( rhs._shapeType ),
    _mass( rhs._mass ),
    _decimatorPercent( rhs._decimatorPercent ),
    _decimatorMaxError( rhs._decimatorMaxError ),
    _decimatorIgnoreBoundaries( rhs._decimatorIgnoreBoundaries ),
    _simplifyPercent( rhs._simplifyPercent ),
    _vertexAggMaxVerts( rhs._vertexAggMaxVerts ),
    _vertexAggMinCellSize( rhs._vertexAggMinCellSize ),
    _reducerGroupThreshold( rhs._reducerGroupThreshold ),
    _reducerMaxEdgeError( rhs._reducerMaxEdgeError ),
    _overall( rhs._overall ),
    _nodeName( rhs._nodeName ),
    _axis( rhs._axis )
{
}


