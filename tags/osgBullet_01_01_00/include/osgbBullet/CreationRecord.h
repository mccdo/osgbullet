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

#ifndef __OSGBBULLET_CREATION_RECORD_H__
#define __OSGBBULLET_CREATION_RECORD_H__ 1


#include <osgbBullet/Export.h>
#include <osgbBullet/CollisionShapes.h>

#include <osg/Object>
#include <osg/Vec3>

#include <btBulletDynamicsCommon.h>

#include <string>



namespace osgbBullet {



// Fill in this struct and pass it to the OSGToCollada constructor
// as a one-step config process, or first configure OSGToCollada
// and then get this record from it, and store it as UserData on
// the rigid body subgraph root node to facilitate saving and
// restoring physics state.
struct OSGBBULLET_EXPORT CreationRecord : public osg::Object
{
    CreationRecord();
    CreationRecord( const CreationRecord& rhs, osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );

    META_Object(osgbBullet,CreationRecord);

    osg::Node* _sceneGraph;

    unsigned int _version;

    osg::Vec3 _com;
    bool _comSet;
    osg::Vec3 _scale;
    BroadphaseNativeTypes _shapeType;
    float _mass;

    // Reserved for future use.
    float _decimatorPercent;
    float _decimatorMaxError;
    bool  _decimatorIgnoreBoundaries;
    float _simplifyPercent;
    unsigned int _vertexAggMaxVerts;
    osg::Vec3 _vertexAggMinCellSize;
    float _reducerGroupThreshold;
    float _reducerMaxEdgeError;
    // END Reserved for future use.

    bool _overall;
    std::string _nodeName;
    osgbBullet::AXIS _axis;
};


// namespace osgbBullet
}

// __OSGBBULLET_CREATION_RECORD_H__
#endif
