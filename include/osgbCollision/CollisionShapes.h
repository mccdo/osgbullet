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

#ifndef __OSGCOLLISION_COLLISIONSHAPES_H__
#define __OSGCOLLISION_COLLISIONSHAPES_H__ 1


#include <osgbCollision/Export.h>
#include <osg/Node>
#include <osg/BoundingBox>

#include <btBulletCollisionCommon.h>


namespace osgbCollision
{


/** \defgroup collisionshapes Collision Shapes
\brief Low-level convenience routines for converting between OSG geometric data and Bullet collision shapes.
*/
/**@{*/

/** \brief Enumerant to denote the major axis of a cylinder.
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

/** \brief Return a Bullet collision shape that approximates the specified OSG geometry. */
OSGBCOLLISION_EXPORT btSphereShape*             btSphereCollisionShapeFromOSG( osg::Node* node );
OSGBCOLLISION_EXPORT btBoxShape*                btBoxCollisionShapeFromOSG( osg::Node* node, const osg::BoundingBox* bb=NULL );
OSGBCOLLISION_EXPORT btCylinderShape*           btCylinderCollisionShapeFromOSG( osg::Node*, AXIS axis = Y );
OSGBCOLLISION_EXPORT btTriangleMeshShape*       btTriMeshCollisionShapeFromOSG( osg::Node * );
OSGBCOLLISION_EXPORT btConvexTriangleMeshShape* btConvexTriMeshCollisionShapeFromOSG( osg::Node * );
OSGBCOLLISION_EXPORT btConvexHullShape*         btConvexHullCollisionShapeFromOSG( osg::Node * );


/** \brief Return an OSG representation of the given bullet collision shape. */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btBoxShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btSphereShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btCylinderShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btConvexTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* btShape, const btTransform& trans = btTransform::getIdentity() );


/**@}*/


// osgbCollision
}


// __OSGCOLLISION_COLLISIONSHAPES_H__
#endif
