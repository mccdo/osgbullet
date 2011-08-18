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

#ifndef __OSGBDYNAMICS_RIGIDBODY_H__
#define __OSGBDYNAMICS_RIGIDBODY_H__ 1

#include <osg/Referenced>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <osgbDynamics/Export.h>

namespace osgbDynamics {

/** RefRigidBody RefRigidBody.h <osgbDynamics\RefRigidBody.h>
\brief A reference counted btRigidBody.

RefRigidBody allows btRigidBody to be added as UserData to an OSG Node.
\note Does \b not delete the rigid body in the destructor (could still be in use by Bullet).
*/
class OSGBDYNAMICS_EXPORT RefRigidBody : public osg::Referenced
{
public:
    RefRigidBody( void );
    RefRigidBody( btRigidBody* rigidBody );

    void setRigidBody( btRigidBody* rigidBody )
    {
        _rigidBody = rigidBody;
    }

    btRigidBody* getRigidBody()
    {
        return( _rigidBody );
    }
    const btRigidBody* getRigidBody() const
    {
        return( _rigidBody );
    }

protected:
    virtual ~RefRigidBody( void );

    btRigidBody* _rigidBody;
};


// osgbDynamics
}


// __OSGBDYNAMICS_RIGIDBODY_H__
#endif
