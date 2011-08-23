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

#include <osgbCollision/ComputeShapeVisitor.h>
#include <osg/ComputeBoundsVisitor>
#include <osg/Geode>
#include <osg/Notify>
#include <osgbCollision/Utils.h>


namespace osgbCollision
{


ComputeShapeVisitor::ComputeShapeVisitor( const BroadphaseNativeTypes shapeType,
    const osgbCollision::AXIS axis, osg::NodeVisitor::TraversalMode traversalMode )
  : osg::NodeVisitor( traversalMode ),
    _shapeType( shapeType ),
    _axis( axis ),
    _shape( NULL )
{
    _shape = new btCompoundShape();
}

void ComputeShapeVisitor::apply( osg::Geode& node )
{
    osg::notify( osg::DEBUG_INFO ) << "Geode" << std::endl;

    createAndAddShape( node );
}

btCollisionShape* ComputeShapeVisitor::getShape()
{
    return( _shape );
}
const btCollisionShape* ComputeShapeVisitor::getShape() const
{
    return( _shape );
}

void ComputeShapeVisitor::createAndAddShape( osg::Node& node )
{
    osg::notify( osg::DEBUG_INFO ) << "In createAndAddShape" << std::endl;

    btCollisionShape* child = createShape( node );
    if( child )
    {
        btCompoundShape* master = static_cast< btCompoundShape* >( _shape );
        btTransform transform; transform.setIdentity();
        master->addChildShape( transform, child );
    }
}
btCollisionShape* ComputeShapeVisitor::createShape( osg::Node& node )
{
    osg::notify( osg::DEBUG_INFO ) << "In createShape" << std::endl;

    btCollisionShape* collision( NULL );
    osg::Vec3 center;

    switch( _shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
    {
        osg::ComputeBoundsVisitor cbv;
        node.accept( cbv );
        osg::BoundingBox bb = cbv.getBoundingBox();
        center = bb.center();
        collision = osgbCollision::btBoxCollisionShapeFromOSG( &node, &bb );
        break;
    }
    case SPHERE_SHAPE_PROXYTYPE:
    {
        osg::BoundingSphere bs = node.getBound();
        center = bs.center();
        collision = osgbCollision::btSphereCollisionShapeFromOSG( &node );
        break;
    }
    case CYLINDER_SHAPE_PROXYTYPE:
    {
        osg::BoundingSphere bs = node.getBound();
        center = bs.center();
        collision = osgbCollision::btCylinderCollisionShapeFromOSG( &node, _axis );
        break;
    }
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        collision = osgbCollision::btTriMeshCollisionShapeFromOSG( &node );
        break;
    }
    case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        collision = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( &node );
        break;
    }
    case CONVEX_HULL_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        collision = osgbCollision::btConvexHullCollisionShapeFromOSG( &node );
        break;
    }
    default:
    {
        osg::notify( osg::FATAL ) << "OSGToCollada: Error, unknown shape type, using tri mesh." << std::endl;
        break;
    }
    }

    if( collision && ( center != osg::Vec3( 0., 0., 0. ) ) )
    {
        btTransform trans; trans.setIdentity();
        trans.setOrigin( osgbCollision::asBtVector3( center ) );
        btCompoundShape* masterShape = new btCompoundShape();
        masterShape->addChildShape( trans, collision );
        collision = masterShape;
    }

    return( collision );
}


// osgbCollision
}
