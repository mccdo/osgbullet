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


/** \brief Creates a compound collision shape and rigid body from the CreationRecord data.

Uses the ComputeShapeVisitor to create a btCompoundShape from the CreationRecord's scene graph.
Currently, a shape per Geode is created. CreationRecord specifies the shape type created per Geode.
*/
OSGBDYNAMICS_EXPORT btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr );

/** \brief Creates a rigid body from the collision shape and CreationRecord data.
*/
OSGBDYNAMICS_EXPORT btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr, btCollisionShape* shape );


// osgbDynamics
}


// __OSGBDYNAMICS_RIGID_BODY_H__
#endif
