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

#ifndef __OSGBBULLET_COLLISIONSHAPES_H__
#define __OSGBBULLET_COLLISIONSHAPES_H__


#include <osgbBullet/Export.h>
#include <osg/Node>
#include <osg/BoundingBox>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

namespace osg {
    class Geode;
}

namespace osgbBullet {


/* Several utility functions to assist with creation of
   Bullet collision shapes from OSG geometry.
*/
enum AXIS
{
    X,
    Y,
    Z,
    _X,
    _Y,
    _Z
};

OSGBBULLET_EXPORT btSphereShape*             btSphereCollisionShapeFromOSG( osg::Node* node );
OSGBBULLET_EXPORT btBoxShape*                btBoxCollisionShapeFromOSG( osg::Node* node, const osg::BoundingBox* bb=NULL );
OSGBBULLET_EXPORT btCylinderShape*           btCylinderCollisionShapeFromOSG( osg::Node*, AXIS axis = Y );
OSGBBULLET_EXPORT btTriangleMeshShape*       btTriMeshCollisionShapeFromOSG( osg::Node * );
OSGBBULLET_EXPORT btConvexTriangleMeshShape* btConvexTriMeshCollisionShapeFromOSG( osg::Node * );
OSGBBULLET_EXPORT btConvexHullShape*         btConvexHullCollisionShapeFromOSG( osg::Node * );


/* Returns an OSG representation of the given bullet collision shape.
*/
OSGBBULLET_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBBULLET_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btBoxShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBBULLET_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btSphereShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBBULLET_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btCylinderShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBBULLET_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBBULLET_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btConvexTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBBULLET_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* btShape, const btTransform& trans = btTransform::getIdentity() );

OSGBBULLET_EXPORT osg::Node* generateGroundPlane( const osg::Vec4& plane, btDynamicsWorld* bulletWorld );

} // end namespace osgbBullet

#endif // __OSGBBULLET_COLLISIONSHAPES_H__
