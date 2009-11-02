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

#ifndef __OSGBBULLET_MOTIONSTATE_H__
#define __OSGBBULLET_MOTIONSTATE_H__

#include <osg/MatrixTransform>

#include <osgwTools/AbsoluteModelTransform.h>

#include <btBulletCollisionCommon.h>
#include <osgbBullet/Export.h>

#include <vector>


namespace osgbBullet {



// Derive a struct from MotionStateCallback and add it to the MotionState class.
// The operator() method will get called whenever Buller sets the world transform.
struct OSGBBULLET_EXPORT MotionStateCallback
{
    MotionStateCallback() {}
    ~MotionStateCallback() {}

    virtual void operator()( const btTransform& worldTrans ) = 0;
};
typedef std::vector< MotionStateCallback* > MotionStateCallbackList;


// forward declaration
class TripleBuffer;

/*!
    A btMotionState that allows Bullet to set the ransformation
    of an OSG subgraph corresponding to a rigid body.
    
    This class can interface with an osg::MatrixTransform
    or an osgwTools::AbsoluteModelTransform.

    Typical usage:
     - Call setTransform() to attach the root node of a subgraph.
       The node must be a MatrixTransform or AbsoluteModelTransform.
     - Call setParentTransform() to specify the initial transformation
       for the subgraph.
     - Call setCenterOfMass() to specify the xyz point corresponding
       to the origin of the Bullet collision shape used by the rigid body.
*/

class OSGBBULLET_EXPORT MotionState : public btMotionState
{
public:
    MotionState( const osg::Matrix& parentTransform = osg::Matrix::identity(),
        const osg::Vec3& centerOfMass = osg::Vec3( 0., 0., 0. ) );
    virtual ~MotionState( void ) { }


    // Bullet interface routines. To be used by Bullet only.
    // Note that setWorldTransform is called by resetTransform.
    virtual void setWorldTransform(const btTransform& worldTrans);
    virtual void getWorldTransform(btTransform& worldTrans ) const;

    void setWorldTransformInternal( const btTransform& worldTrans );


    // Attach a subgraph that corresponds to the rigid body owning
    // this MotionState. MotionState removes the center of mass
    // offset from its transformation before driving this subgraph.
    void setTransform( osg::Transform* transform );
    osg::Transform* getTransform();
    const osg::Transform* getTransform() const;

    // Set the initial transformation for the subgraph. Typically,
    // this is the accumulated model transformations of ancestor nodes.
    void setParentTransform( const osg::Matrix m );
    osg::Matrix getParentTransform() const;

    // Set the center of mass. This is an xyz point in the subgraph's
    // local coordinates that corresponds to the origin in the
    // collision shape's local coordinates.
    void setCenterOfMass( const osg::Vec3& com );
    osg::Vec3 getCenterOfMass() const;

    void setScale( const osg::Vec3& scale );
    osg::Vec3 getScale() const;

    // Allows application code to get modified when Bullet updates the
    // position of a rigid body.
    MotionStateCallbackList& getCallbackList();

    // This is a convenience routine that calls setWorldTransform with
    // the concatenation of the center of mass and parent transform.
    // It is called by setCenterOfMass and setParentTransform to set the
    // initial world transformation. See setWorldTransformation for more
    // details.
    // Apps can call this method directly to invoke setWorldTransform() with
    // the concatentation of the center of mass and parent transforms.
    void resetTransform();


    // TripleBuffer support. The following methods allows MotionState
    // to keep its btTransform world transform mechanism in a TripleBuffer
    // objects, which enables multithreaded physics simulation.
    //
    // Call to initially register the MotionState with the TripleBuffer.
    void registerTripleBuffer( osgbBullet::TripleBuffer* tb );
    //
    // Call to get the latest world transform value from the TripleBuffer
    // and push it out to the OSG Transform.
    void updateTripleBuffer( const char* addr );

private:
    // One or the other of these will be valid, depending on whether the
    // MotionState is associated with an AbsoluteModelTransform or a
    // plain old MatrixTransform.
    osg::ref_ptr< osg::MatrixTransform > _mt;
    osg::ref_ptr< osgwTools::AbsoluteModelTransform > _amt;

    // This is the accumulated model-to-world matrix of parent Transform nodes
    // in the scene graph.
    osg::Matrix _parentTransform;
    // _com is used to align the origin-centered collision shape with
    // an off-origin OSG visual representation.
    osg::Vec3 _com;

    osg::Vec3 _scale;

    // This is the transformation of the collision shape / rigid body within the Bullet physics simulation.
    // See setWorldTransformation for more details.
    btTransform _transform;

    MotionStateCallbackList _mscl;

    // TripleBuffer support.
    TripleBuffer* _tb;
    unsigned int _tbIndex;
};


// TripleBuffer support. Apps running Bullet is a thread separate from OSG
// rendering should keep a list of all MotionState objects. During update,
// the app should call TripleBufferMotionStateUpdate to update all MotionState
// objects with data from the TripleBuffer (and push those matrices out to
// the OSG scene graph Transform nodes).
typedef std::vector< osgbBullet::MotionState* > MotionStateList;

bool OSGBBULLET_EXPORT TripleBufferMotionStateUpdate( osgbBullet::MotionStateList& msl, osgbBullet::TripleBuffer* tb );


} // namespace osgbBullet

#endif // __OSGBBULLET_MOTIONSTATE_H__
