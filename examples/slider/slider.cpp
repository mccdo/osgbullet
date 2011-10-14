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
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgUtil/Optimizer>

#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/LaunchHandler.h>
#include <osgbInteraction/SaveRestoreHandler.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



// Filter out collisions between the drawer and nightstand.
//
// Bullet collision filtering tutorial:
//   http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Collision_Filtering
//
// Define filter groups
enum CollisionTypes {
    COL_DRAWER = 0x1 << 0,
    COL_STAND = 0x1 << 1,
    COL_DEFAULT = 0x1 << 2,
};
// Define filter masks
unsigned int drawerCollidesWith( COL_DEFAULT );
unsigned int standCollidesWith( COL_DEFAULT );
unsigned int defaultCollidesWith( COL_DRAWER | COL_STAND | COL_DEFAULT );



btRigidBody* standBody;
void makeStaticObject( btDiscreteDynamicsWorld* bw, osg::Node* node, const osg::Matrix& m )
{
    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = node;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = 0.f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb, COL_STAND, standCollidesWith );

    // Save RB in global
    standBody = rb;
}

btRigidBody* drawerBody;
osg::Transform* makeDrawer( btDiscreteDynamicsWorld* bw, osgbInteraction::SaveRestoreHandler* srh, osg::Node* node, const osg::Matrix& m )
{
    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    osgwTools::insertAbove( node, amt );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->setCenterOfMass( node->getBound().center() );
    cr->_parentTransform = m;
    cr->_mass = .75f;
    cr->_restitution = .5f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb, COL_DRAWER, drawerCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, as AMT UserData (for DragHandler), and in SaveRestoreHandler.
    drawerBody = rb;
    amt->setUserData( new osgbCollision::RefRigidBody( rb ) );
    srh->add( "gate", rb );

    return( amt );
}


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


osg::Node* findNamedNode( osg::Node* model, const std::string& name, osg::Matrix& xform )
{
    osgwTools::FindNamedNode fnn( name );
    model->accept( fnn );
    if( fnn._napl.empty() )
    {
        osg::notify( osg::FATAL ) << "hinge: Can't find node names \"" << name << "\"." << std::endl;
        return( NULL );
    }
    xform = osg::computeLocalToWorld( fnn._napl[ 0 ].second );
    return( fnn._napl[ 0 ].first );
}

int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::Group* root = new osg::Group;

    osg::Group* launchHandlerAttachPoint = new osg::Group;
    root->addChild( launchHandlerAttachPoint );


    osg::ref_ptr< osg::Node > rootModel = osgDB::readNodeFile( "NightStand.flt" );
    if( !rootModel.valid() )
    {
        osg::notify( osg::FATAL ) << "hinge: Can't load data file \"NightStand.flt\"." << std::endl;
        return( 1 );
    }

    root->addChild( rootModel.get() );

    // Get Node pointers and parent transforms for the night stand and drawer.
    // (Node names are taken from the osgWorks osgwnames utility.)
    osg::Matrix standXform, drawerXform;
    osg::Node* standNode = findNamedNode( rootModel.get(), "NightStand_Body", standXform );
    osg::Node* drawerNode = findNamedNode( rootModel.get(), "DOF_Drawer", drawerXform );
    if( ( standNode == NULL ) || ( drawerNode == NULL ) )
        return( 1 );

    // Open the drawer slightly.
    // TBD this is a hardcoded axis/distance.
    drawerXform *= osg::Matrix::translate( 0., -.2, 0. );

    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new
        osgbInteraction::SaveRestoreHandler;

    // Make Bullet rigid bodies and collision shapes for the drawer...
    makeDrawer( bulletWorld, srh, drawerNode, drawerXform );
    // ...and the stand.
    makeStaticObject( bulletWorld, standNode, standXform );


    // Add ground
    const osg::Vec4 plane( 0., 0., 1., 0. );
    root->addChild( osgbDynamics::generateGroundPlane( plane,
        bulletWorld, NULL, COL_DEFAULT, defaultCollidesWith ) );
    

    // create slider constraint between drawer and groundplane and add it to world
    // Note: Bullet slider is always along x axis. Alter this behavior with reference frames.
    {
        // Model-specific constants.
        // TBD Should obtain these from model metadata or user input:
        const osg::Vec3 drawerAxis( 0., 1., 0. );
        const float drawerMinLimit( -1.f );
        const float drawerMaxLimit( 0.f );


        // Compute a matrix that transforms the stand's collision shape origin and x axis
        // to the drawer's origin and drawerAxis.
        //   1. Matrix to align the (slider constraint) x axis with the drawer axis.
        const osg::Vec3 bulletSliderAxis( 1., 0., 0. );
        const osg::Matrix axisRotate( osg::Matrix::rotate( bulletSliderAxis, drawerAxis ) );
        //
        //   2. Inverse stand center of mass offset.
        osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( standBody->getMotionState() );
        const osg::Matrix invStandCOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
        //
        //   3. Transform from the stand's origin to the drawer's origin.
        const osg::Matrix standToDrawer( osg::Matrix::inverse( standXform ) * drawerXform );
        //
        //   4. The final stand frame matrix.
        btTransform standFrame = osgbCollision::asBtTransform(
            axisRotate * invStandCOM * standToDrawer );


        // Compute a matrix that transforms the drawer's collision shape origin and x axis
        // to the drawer's origin and drawerAxis.
        //   1. Drawer center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( drawerBody->getMotionState() );
        const osg::Matrix invDrawerCOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
        //
        //   2. The final drawer frame matrix.
        btTransform drawerFrame = osgbCollision::asBtTransform(
            axisRotate * invDrawerCOM );


        btSliderConstraint* slider = new btSliderConstraint( *drawerBody, *standBody, drawerFrame, standFrame, false );
        slider->setLowerLinLimit( drawerMinLimit );
	    slider->setUpperLinLimit( drawerMaxLimit );
        bulletWorld->addConstraint( slider, true );
    }


    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 1., -7., 2. ), osg::Vec3( 0., 0., 1. ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    // Create the launch handler.
    osgbInteraction::LaunchHandler* lh = new osgbInteraction::LaunchHandler(
        bulletWorld, launchHandlerAttachPoint, viewer.getCamera() );
    {
        // Use a custom launch model: A scaled-down teapot.
        osg::ref_ptr< osg::MatrixTransform > mt = new osg::MatrixTransform(
            osg::Matrix::scale( 0.2, 0.2, 0.2 ) );
        mt->addChild( osgDB::readNodeFile( "teapot.osg" ) );
        osgUtil::Optimizer opt;
        opt.optimize( mt.get(), osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS );
        mt->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );

        lh->setLaunchModel( mt.get() );
        lh->setInitialVelocity( 20. );

        // Also add the proper collision masks
        lh->setCollisionFlags( COL_DEFAULT, defaultCollidesWith );

        viewer.addEventHandler( lh );
    }

    srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh );
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


/** \page sliderlowlevel Simple Slider Constraint

Demonstrates coding directly to the Bullet API to create a slider constraint.

Use the --debug command line option to enable debug collision object display.

\section slidercontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.
\li shift-leftmouse: Launches a sphere into the scene.

*/
