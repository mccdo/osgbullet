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
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/ComputeBoundsVisitor>

#include <osg/Light>
#include <osg/LightSource>
#include <osg/LightModel>

#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/Constraints.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/SaveRestoreHandler.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>
#include <osgwTools/Version.h>

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

void simpleLighting( osg::Group* root )
{
    osg::StateSet* rootState = root->getOrCreateStateSet();
    rootState->setMode( GL_LIGHT0, osg::StateAttribute::ON );

    osg::LightSource* ls = new osg::LightSource();
    ls->setReferenceFrame( osg::LightSource::RELATIVE_RF );
    root->addChild( ls );

    osg::Light* light = new osg::Light;
    light->setLightNum( 0 );
    light->setAmbient( osg::Vec4( 1., 1., 1., 1. ) );
    light->setDiffuse( osg::Vec4( 1., 1., 1., 1. ) );
    light->setSpecular( osg::Vec4( 1., 1., 1., 1. ) );

    osg::Vec3 pos( -.5, -.4, 2. );
    light->setPosition( osg::Vec4( pos, 1. ) );
    ls->setLight( light );

    osg::LightModel* lm = new osg::LightModel;
    lm->setAmbientIntensity( osg::Vec4( 0., 0., 0., 1. ) );
    lm->setColorControl( osg::LightModel::SEPARATE_SPECULAR_COLOR );
    rootState->setAttribute( lm, osg::StateAttribute::ON );
}


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::Group* root = new osg::Group;

    simpleLighting( root );


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

    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new
        osgbInteraction::SaveRestoreHandler;

    // Make Bullet rigid bodies and collision shapes for the drawer...
    makeDrawer( bulletWorld, srh.get(), drawerNode, drawerXform );
    // ...and the stand.
    makeStaticObject( bulletWorld, standNode, standXform );


    // create slider constraint between drawer and stand and add it to world
    // Note: Bullet slider is always along x axis. Alter this behavior with reference frames.
    btSliderConstraint* slider;
    {
        // Model-specific constants.
        // TBD Should obtain these from model metadata or user input:
        const osg::Vec3 drawerAxis( 0., 1., 0. );
        const float drawerMaxLimit( 0.f );

        osg::ComputeBoundsVisitor cbv;
        drawerNode->accept( cbv );
        const osg::BoundingBox& bb = cbv.getBoundingBox();
        float drawerMinLimit = -( bb.yMax() - bb.yMin() );

        osg::ref_ptr< osgbDynamics::SliderConstraint > sc = new osgbDynamics::SliderConstraint(
            drawerBody, drawerXform, standBody, standXform, drawerAxis,
            osg::Vec2( drawerMinLimit, drawerMaxLimit ) );

        slider = sc->getAsBtSlider();
        bulletWorld->addConstraint( slider, true );

        osgDB::writeObjectFile( *sc, "test.osg" );

        osg::ref_ptr< osgbDynamics::SliderConstraint > sc2 = dynamic_cast<
            osgbDynamics::SliderConstraint* >( osgDB::readObjectFile( "test.osg" ) );
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
    tb->setHomePosition( osg::Vec3( .8, -5., 1.6 ), osg::Vec3( 0., 0., .5 ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    srh->capture();
    viewer.addEventHandler( srh.get() );
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
