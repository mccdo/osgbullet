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

#ifndef __OSGBCOLLISION_REF_COLLISION_OBJECT_H__
#define __OSGBCOLLISION_REF_COLLISION_OBJECT_H__ 1

#include <osgbCollision/Export.h>
#include <osg/Referenced>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>


namespace osgbCollision {


/** \class RefCollisionObject RefCollisionObject.h <osgbCollision/RefCollisionObject.h>
\brief A reference-counted btCollisionObject (allows it to be added as UserData to an OSG Node).
*/
class OSGBCOLLISION_EXPORT RefCollisionObject : public osg::Referenced
{
public:
    RefCollisionObject( void );
    RefCollisionObject( btCollisionObject* collisionObject );

    void setCollisionObject( btCollisionObject* collisionObject )
    {
        _collisionObject = collisionObject;
    }

    btCollisionObject* getCollisionObject()
    {
        return( _collisionObject );
    }
    const btCollisionObject* getCollisionObject() const
    {
        return( _collisionObject );
    }

protected:
    virtual ~RefCollisionObject( void );

    btCollisionObject* _collisionObject;
};


// namespace osgbCollision
}


//__OSGBCOLLISION_REF_COLLISION_OBJECT_H__
#endif
