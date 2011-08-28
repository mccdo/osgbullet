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

#ifndef __OSGBDYNAMICS_RIGID_BODY_H__
#define __OSGBDYNAMICS_RIGID_BODY_H__ 1

#include <osgbDynamics/Export.h>
#include <osgbDynamics/CreationRecord.h>
#include <btBulletDynamicsCommon.h>


namespace osgbDynamics
{


/** \defgroup rigidbody Rigid Body Creation
\brief Convenience routines for creating Bullet rigid bodies from scene graphs.

Note that these are merely convenience routines. They are based on the CreationRecord class,
which supports the dot OSG file format. If you have no need for the CreationRecord, you can
interface directly with Bullet to create the rigid bodies.

These routines also implicitly create a MotionState to keep the OSG visual representation
in sync with the Bullet physics representation. In order for this to work correctly, the
root node of the subgraph stored in CreationRecord::_sceneGraph must be either an
osg::MatrixTransform or an osgwTools::AbsoluteModelTransform (from the osgWorks project).
*/
/*@{*/


/** \brief Creates a compound collision shape and rigid body from the CreationRecord data.

Uses the ComputeShapeVisitor to create a btCompoundShape from the CreationRecord's scene graph.
Currently, a shape per Geode is created. CreationRecord specifies the shape type created per Geode.
*/
OSGBDYNAMICS_EXPORT btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr );

/** \overload
<p>
Use this function to create a rigid body if you have already created the collision shape.
This is useful for sharing collision shapes.
</p>
*/
OSGBDYNAMICS_EXPORT btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr, btCollisionShape* shape );


/**@}*/


// osgbDynamics
}


// __OSGBDYNAMICS_RIGID_BODY_H__
#endif
