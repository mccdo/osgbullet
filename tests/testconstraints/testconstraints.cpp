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

#include "ctest.h"

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/Constraints.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/SaveRestoreHandler.h>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );

    return( dynamicsWorld );
}


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    if( arguments.find( "--ctest" ) > 0 )
        return( runCTest() );

    const bool debugDisplay( arguments.find( "--debug" ) > 0 );


    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::Group* root = new osg::Group;

    // Add ground
    const osg::Vec4 plane( 0., 0., 1., -1.5 );
    osg::Node* groundRoot = osgbDynamics::generateGroundPlane( plane, bulletWorld );
    root->addChild( groundRoot );

    if( arguments.find( "PlaneSlider" ) > 0 )
    {
    }
    else if( arguments.find( "BoxSlider" ) > 0 )
    {
    }
    else // SliderConstraint by default.
    {
        osg::Node* node = osgDB::readNodeFile( "tetra.osg" );
        if( node == NULL )
            return( 1 );
        osg::Matrix aXform = osg::Matrix::translate( 4., 2., 0. );

        osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
        amt->setDataVariance( osg::Object::DYNAMIC );
        amt->addChild( node );
        root->addChild( amt );

        osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
        cr->_sceneGraph = amt;
        cr->_shapeType = BOX_SHAPE_PROXYTYPE;
        cr->_mass = 0.5;
        cr->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( cr.get() );
        amt->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        node = osgDB::readNodeFile( "block.osg" );
        if( node == NULL )
            return( 1 );
        osg::Matrix bXform = osg::Matrix::identity();

        amt = new osgwTools::AbsoluteModelTransform;
        amt->setDataVariance( osg::Object::DYNAMIC );
        amt->addChild( node );
        root->addChild( amt );

        cr = new osgbDynamics::CreationRecord;
        cr->_sceneGraph = amt;
        cr->_shapeType = BOX_SHAPE_PROXYTYPE;
        cr->_mass = 4.;
        cr->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( cr.get() );
        amt->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        {
            osg::Vec3 axis( 0., 0., 1. );
            osg::Vec2 limits( -4., 4. );
            osg::ref_ptr< osgbDynamics::SliderConstraint > sc = new osgbDynamics::SliderConstraint(
                rbA, aXform, rbB, bXform, axis, limits );
            bulletWorld->addConstraint( sc->getConstraint() );
        }
    }


    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        osg::Node* dbgRoot = dbgDraw->getSceneGraph();
        root->addChild( dbgRoot );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 800, 450 );
    viewer.setSceneData( root );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -18., 3. ), osg::Vec3( 0., 0., 1.5 ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    viewer.addEventHandler( new osgbInteraction::DragHandler(
        bulletWorld, viewer.getCamera() ) );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

    return( 0 );
}
