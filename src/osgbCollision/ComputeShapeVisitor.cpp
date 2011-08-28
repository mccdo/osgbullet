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
#include <osgwTools/Transform.h>
#include <osgbCollision/Utils.h>
#include <osgwTools/AbsoluteModelTransform.h>


namespace osgbCollision
{


ComputeShapeVisitor::ComputeShapeVisitor( const BroadphaseNativeTypes shapeType,
    const osgbCollision::AXIS axis, osg::NodeVisitor::TraversalMode traversalMode )
  : osg::NodeVisitor( traversalMode ),
    _shapeType( shapeType ),
    _axis( axis ),
    _shape( new btCompoundShape() )
{
}

void ComputeShapeVisitor::apply( osg::Transform& node )
{
    const bool nonAMT = ( dynamic_cast< osgwTools::AbsoluteModelTransform* >( &node ) == NULL );
    if( nonAMT )
        _localNodePath.push_back( &node );

    traverse( node );

    if( nonAMT )
        _localNodePath.pop_back();
}

void ComputeShapeVisitor::apply( osg::Geode& node )
{
    osg::notify( osg::DEBUG_INFO ) << "Geode" << std::endl;

    osg::Matrix m = osg::computeLocalToWorld( _localNodePath );
    createAndAddShape( node, m );
}

btCollisionShape* ComputeShapeVisitor::getShape()
{
    return( _shape );
}
const btCollisionShape* ComputeShapeVisitor::getShape() const
{
    return( _shape );
}

void ComputeShapeVisitor::createAndAddShape( osg::Node& node, const osg::Matrix& m )
{
    osg::notify( osg::DEBUG_INFO ) << "In createAndAddShape" << std::endl;

    btCollisionShape* child = createShape( node, m );
    if( child )
    {
        btCompoundShape* master = static_cast< btCompoundShape* >( _shape );
        btTransform transform; transform.setIdentity();
        master->addChildShape( transform, child );
    }
}
btCollisionShape* ComputeShapeVisitor::createShape( osg::Node& node, const osg::Matrix& m )
{
    osg::notify( osg::DEBUG_INFO ) << "In createShape" << std::endl;

    // Make a copy of the incoming node and its date, then
    // transform the copy by the specified matrix.
    osg::Node* nodeCopy;
    if( node.asGeode() != NULL )
    {
        nodeCopy = new osg::Geode( *( node.asGeode() ), osg::CopyOp::DEEP_COPY_ALL );
        osgwTools::transform( m, nodeCopy->asGeode() );
    }
    else
    {
        osg::notify( osg::WARN ) << "ComputeShapeVisitor encountered non-Geode." << std::endl;
        return( NULL );
    }

    btCollisionShape* collision( NULL );
    osg::Vec3 center;

    switch( _shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
    {
        osg::ComputeBoundsVisitor cbv;
        nodeCopy->accept( cbv );
        osg::BoundingBox bb = cbv.getBoundingBox();
        center = bb.center();
        collision = osgbCollision::btBoxCollisionShapeFromOSG( nodeCopy, &bb );
        break;
    }
    case SPHERE_SHAPE_PROXYTYPE:
    {
        osg::BoundingSphere bs = nodeCopy->getBound();
        center = bs.center();
        collision = osgbCollision::btSphereCollisionShapeFromOSG( nodeCopy );
        break;
    }
    case CYLINDER_SHAPE_PROXYTYPE:
    {
        osg::BoundingSphere bs = nodeCopy->getBound();
        center = bs.center();
        collision = osgbCollision::btCylinderCollisionShapeFromOSG( nodeCopy, _axis );
        break;
    }
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        collision = osgbCollision::btTriMeshCollisionShapeFromOSG( nodeCopy );
        break;
    }
    case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        collision = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( nodeCopy );
        break;
    }
    case CONVEX_HULL_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        collision = osgbCollision::btConvexHullCollisionShapeFromOSG( nodeCopy );
        break;
    }
    default:
    {
        osg::notify( osg::FATAL ) << "ComputeShapeVisitor: Error, unknown shape type, using tri mesh." << std::endl;
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
