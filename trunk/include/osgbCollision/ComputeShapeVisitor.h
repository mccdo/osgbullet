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

#ifndef __OSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__
#define __OSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__ 1

#include <osgbCollision/Export.h>
#include <osgbCollision/CollisionShapes.h>
#include <btBulletCollisionCommon.h>
#include <osg/NodeVisitor>
#include <osgwTools/Version.h>

#include <string>


namespace osgbCollision
{


/** \class ComputeShapeVisitor ComputeShapeVisitor.h <osgbCollision/ComputeShapeVisitor.h>
\brief TBD
*/
class OSGBCOLLISION_EXPORT ComputeShapeVisitor : public osg::NodeVisitor
{
public:
    ComputeShapeVisitor( const BroadphaseNativeTypes shapeType, const osgbCollision::AXIS axis=Y,
        osg::NodeVisitor::TraversalMode traversalMode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );

#if( OSGWORKS_OSG_VERSION >= 20800 )
    META_NodeVisitor(osgbCollision,ComputeShapeVisitor);
#endif

    void apply( osg::Geode& node );

    btCollisionShape* getShape();
    const btCollisionShape* getShape() const;

protected:
    void createAndAddShape( osg::Node& node );
    btCollisionShape* createShape( osg::Node& node );

    const BroadphaseNativeTypes _shapeType;
    const osgbCollision::AXIS _axis;

    btCollisionShape* _shape;
};


// osgbCollision
}


// __OSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__
#endif
