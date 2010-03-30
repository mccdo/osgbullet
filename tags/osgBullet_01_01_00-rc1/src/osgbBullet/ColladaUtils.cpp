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

#include <btBulletDynamicsCommon.h>
#include <BulletColladaConverter/ColladaConverter.h>

#include <osgbBullet/MotionState.h>
#include <osgbBullet/RefRigidBody.h>
#include <osgbBullet/CollisionShapes.h>
#include <osgbBullet/ColladaUtils.h>
#include <osgbBullet/Utils.h>

#include <osgwTools/AbsoluteModelTransform.h>

#include <osg/Node>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgDB/FileUtils>
#include <osg/Notify>
#include <osg/io_utils>

namespace osgbBullet
{

btDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

    return( dynamicsWorld );
}

btRigidBody* loadDae( osg::Transform* node, const osg::NodePath& np, const std::string& daeName,
    btDynamicsWorld* dw )
{
    // Debug dump the node path.
    osg::NodePath::const_iterator it;
    for( it=np.begin(); it!=np.end(); it++ )
        osg::notify( osg::INFO ) << (*it)->className() << ", ";
    osg::notify( osg::INFO ) << std::endl;

    std::string fullDaeName( osgDB::findDataFile( daeName ) );
    if( fullDaeName.empty() )
    {
        osg::notify( osg::FATAL ) << "Can't find DAE file: " << daeName << std::endl;
        osg::notify( osg::FATAL ) << "See scripts/mkdae.bat for info on creating DAE files using osgbpp." << std::endl;
        return NULL;
    }
    osg::notify( osg::INFO ) << "Attempting to load DAE file: " << fullDaeName << std::endl;

    osg::BoundingSphere bs = node->getBound();
    osg::Vec3 com( bs._center );
    osg::notify( osg::INFO ) << "COM: " << com << std::endl;

    osg::Matrix parentTrans = osg::computeLocalToWorld( np );

    osg::notify( osg::INFO ) << "PreLoad" << std::endl;
    btDynamicsWorld* lw = initPhysics();
    ColladaConverter* cc = new ColladaConverter( lw );
    // TBD Not available in 2.75 Bullet release, maybe next one.
    //cc->setVerbosity( ColladaConverter::SILENT );
    cc->load( fullDaeName.c_str() );
    btRigidBody* rb = cc->getRigidBody( 0 );
    cc->reset();
    lw->removeRigidBody( rb );
    delete cc;
    delete lw;
    osg::notify( osg::INFO ) << "PostLoad" << std::endl;

    osgbBullet::MotionState* motion = new osgbBullet::MotionState;
    motion->setTransform( node );
    if( dw != NULL )
    {
        dw->addRigidBody( rb );
        osg::notify( osg::INFO ) << "rb grav: " << osgbBullet::asOsgVec3( rb->getGravity() ) << std::endl;
    }

    motion->setCenterOfMass( com );
    motion->setParentTransform( parentTrans );
    rb->setMotionState( motion );

    node->setUserData( new osgbBullet::RefRigidBody( rb ) );

    return( rb );
}

}
