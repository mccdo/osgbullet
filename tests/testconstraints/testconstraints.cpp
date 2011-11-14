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


    osg::Node* nodeA = osgDB::readNodeFile( "tetra.osg" );
    if( nodeA == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtA = new osgwTools::AbsoluteModelTransform;
    amtA->setDataVariance( osg::Object::DYNAMIC );
    amtA->addChild( nodeA );
    root->addChild( amtA );

    osg::ref_ptr< osgbDynamics::CreationRecord > crA = new osgbDynamics::CreationRecord;
    crA->_sceneGraph = amtA;
    crA->_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
    crA->_mass = 0.5;


    osg::Node* nodeB = osgDB::readNodeFile( "block.osg" );
    if( nodeB == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtB = new osgwTools::AbsoluteModelTransform;
    amtB->setDataVariance( osg::Object::DYNAMIC );
    amtB->addChild( nodeB );
    root->addChild( amtB );

    osg::ref_ptr< osgbDynamics::CreationRecord > crB = new osgbDynamics::CreationRecord;
    crB->_sceneGraph = amtB;
    crB->_shapeType = BOX_SHAPE_PROXYTYPE;
    crB->_mass = 4.;


    osg::Node* nodeC = osgDB::readNodeFile( "dice.osg" );
    if( nodeC == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtC = new osgwTools::AbsoluteModelTransform;
    amtC->setDataVariance( osg::Object::DYNAMIC );
    amtC->addChild( nodeC );
    root->addChild( amtC );

    osg::ref_ptr< osgbDynamics::CreationRecord > crC = new osgbDynamics::CreationRecord;
    crC->_sceneGraph = amtC;
    crC->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    crC->_mass = 1.5;


    osg::Node* nodeD = osgDB::readNodeFile( "axes.osg" );
    if( nodeD == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtD = new osgwTools::AbsoluteModelTransform;
    amtD->setDataVariance( osg::Object::DYNAMIC );
    amtD->addChild( nodeD );
    root->addChild( amtD );

    osg::ref_ptr< osgbDynamics::CreationRecord > crD = new osgbDynamics::CreationRecord;
    crD->_sceneGraph = amtD;
    crD->_shapeType = BOX_SHAPE_PROXYTYPE;


    if( arguments.find( "Planar" ) > 0 )
    {
    }
    else if( arguments.find( "Box" ) > 0 )
    {
    }
    else if( arguments.find( "BallAndSocket" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 1., 1., 2.5 );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( 2., 12., 5. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix dXform = osg::Matrix::rotate( .2, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( -9., 0., 0. );
        crD->_parentTransform = dXform;
        btRigidBody* rbD = osgbDynamics::createRigidBody( crD.get() );
        amtD->setUserData( new osgbCollision::RefRigidBody( rbD ) );
        bulletWorld->addRigidBody( rbD );

        {
            osg::Vec3 point( 0., 0., 1.5 );
            osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons0 = new osgbDynamics::BallAndSocketConstraint(
                rbA, aXform, rbB, bXform, point );
            bulletWorld->addConstraint( cons0->getConstraint() );

            point.set( 2., 12., 5. );
            osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons1 = new osgbDynamics::BallAndSocketConstraint(
                rbC, cXform, NULL, osg::Matrix::identity(), point );
            bulletWorld->addConstraint( cons1->getConstraint() );

            // Make point correspond roughly to (1., 0., 1.) in local coords)
            point = osg::Vec3( 1., 0., 1. ) * dXform;
            osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons2 = new osgbDynamics::BallAndSocketConstraint(
                rbD, dXform, NULL, osg::Matrix::identity(), point );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "TwistSlider" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 0., 3. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( 2., 12., 5. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        {
            osg::Vec3 axis( 0., 1., .1 );
            osg::Vec2 limits( -1., 1. );
            osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons0 = new osgbDynamics::TwistSliderConstraint(
                rbA, aXform, rbB, bXform, axis, limits );
            bulletWorld->addConstraint( cons0->getConstraint() );

            axis.set( 1., 1., 0. );
            limits.set( -3., 3. );
            osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons1 = new osgbDynamics::TwistSliderConstraint(
                rbC, cXform, NULL, osg::Matrix::identity(), axis, limits );
            bulletWorld->addConstraint( cons1->getConstraint() );
        }
    }
    else // SliderConstraint by default.
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 0., 3. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( 2., 12., 5. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix dXform = osg::Matrix::rotate( 0.7, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( -9., 0., 0. );
        crD->_parentTransform = dXform;
        btRigidBody* rbD = osgbDynamics::createRigidBody( crD.get() );
        amtD->setUserData( new osgbCollision::RefRigidBody( rbD ) );
        bulletWorld->addRigidBody( rbD );

        {
            osg::Vec3 axis( 0., 1., .1 );
            osg::Vec2 limits( -1., 1. );
            osg::ref_ptr< osgbDynamics::SliderConstraint > cons0 = new osgbDynamics::SliderConstraint(
                rbA, aXform, rbB, bXform, axis, limits );
            bulletWorld->addConstraint( cons0->getConstraint() );

            axis.set( 1., 1., 0. );
            limits.set( -3., 3. );
            osg::ref_ptr< osgbDynamics::SliderConstraint > cons1 = new osgbDynamics::SliderConstraint(
                rbC, cXform, NULL, osg::Matrix::identity(), axis, limits );
            bulletWorld->addConstraint( cons1->getConstraint() );

            axis.set( 0., 1., 0. );
            limits.set( -1., 1. );
            osg::ref_ptr< osgbDynamics::SliderConstraint > cons2 = new osgbDynamics::SliderConstraint(
                rbD, dXform, NULL, osg::Matrix::identity(), axis, limits );
            bulletWorld->addConstraint( cons2->getConstraint() );
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
