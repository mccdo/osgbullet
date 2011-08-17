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

#ifndef __OSGBCOLLISION_REF_COLLISION_SHAPE_H__
#define __OSGBCOLLISION_REF_COLLISION_SHAPE_H__ 1

#include <osgbCollision/Export.h>
#include <osg/Referenced>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>


namespace osgbCollision
{


/** \class RefCollisionShape RefCollisionShape.h <osgbCollision/RefCollisionShape.h>
\brief A reference-counted btCollisionShape (allows it to be added as UserData to an OSG Node).
*/
class OSGBCOLLISION_EXPORT RefCollisionShape : public osg::Referenced
{
public:
    RefCollisionShape( void );
    RefCollisionShape( btCollisionShape* collisionShape );

    void setCollisionShape( btCollisionShape* collisionShape )
    {
        _collisionShape = collisionShape;
    }

    btCollisionShape* getCollisionShape()
    {
        return( _collisionShape );
    }
    const btCollisionShape* getCollisionShape() const
    {
        return( _collisionShape );
    }

protected:
    virtual ~RefCollisionShape( void );

    btCollisionShape* _collisionShape;
};


// osgbCollision
}


// __OSGBCOLLISION_REF_COLLISION_SHAPE_H__
#endif
