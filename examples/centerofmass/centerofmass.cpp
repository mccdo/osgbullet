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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>

#include <osgbDynamics/GroundPlane.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/Utils.h>

#include <btBulletDynamicsCommon.h>


btRigidBody* createObject( osg::Group* parent, const osg::Matrix& m, const osg::Vec3& com=osg::Vec3(0,0,0), bool setCom=false )
{
    osg::Node* node = osgDB::readNodeFile( "com.osg" );
    if( node == NULL )
    {
        osg::notify( osg::FATAL ) << "Can't load file \"com.osg\". Make sure osgBullet data directory is in OSG_FILE_PATH." << std::endl;
        return( NULL );
    }

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    parent->addChild( mt );
    mt->addChild( node );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    if( setCom )
        cr->setCenterOfMass( com );
    cr->_sceneGraph = mt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_parentTransform = m;
    cr->_restitution = 1.f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

    rb->setAngularVelocity( btVector3( 0., .2, 0. ) );

    return( rb );
}


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


int main( int argc, char** argv )
{
    btDynamicsWorld* bw = initPhysics();
    osg::Group* root = new osg::Group;

    osg::Matrix m;
    m = osg::Matrix::rotate( .4, 0., 0., 1. ) * osg::Matrix::translate( -24., 0., 10. );
    bw->addRigidBody( createObject( root, m, osg::Vec3( 0., 0., 0. ), true ) );
    m = osg::Matrix::rotate( .4, 0., 0., 1. ) * osg::Matrix::translate( -4., 0., 10. );
    bw->addRigidBody( createObject( root, m, osg::Vec3( 2.15, 3., 2. ), true ) );
    m = osg::Matrix::rotate( .4, 0., 0., 1. ) * osg::Matrix::translate( 16., 0., 10. );
    bw->addRigidBody( createObject( root, m ) );

    root->addChild( osgbDynamics::generateGroundPlane( osg::Vec4( 0.f, 0.f, 1.f, 0.f ), bw ) );


    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 1024, 640 );
    viewer.setSceneData( root );
    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator( tb );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bw->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;
        viewer.frame();
    }

    return( 0 );
}
