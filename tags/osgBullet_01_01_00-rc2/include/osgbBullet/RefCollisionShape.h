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

#ifndef __OSGBBULLET_REFCOLLISIONSHAPE_H__
#define __OSGBBULLET_REFCOLLISIONSHAPE_H__

#include <osg/Referenced>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>

#include <osgbBullet/Export.h>

namespace osgbBullet {


//A reference-counted btCollisionShape (allows it to be added
// as UserData to an OSG Node).


class OSGBBULLET_EXPORT RefCollisionShape : public osg::Referenced
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

}


//__OSGBBULLET_REFCOLLISIONSHAPE_H__
#endif
