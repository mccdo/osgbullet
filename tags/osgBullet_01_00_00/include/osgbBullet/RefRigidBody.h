
#ifndef __OSGBBULLET_RIGIDBODY_H__
#define __OSGBBULLET_RIGIDBODY_H__

#include <osg/Referenced>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <osgbBullet/Export.h>

namespace osgbBullet {

/*!
    A reference-counted btRigidBody (allows it to be added
    as UserData to an OSG Node).
    Does _not_ delete the rigid body in the destructor (could still be in use by Bullet).
*/

class OSGBBULLET_EXPORT RefRigidBody : public osg::Referenced
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

} /* namespace */

#endif // __OSGBBULLET_RIGIDBODY_H__
