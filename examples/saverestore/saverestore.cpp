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
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>

#include <osgbInteraction/SaveRestoreHandler.h>
#include <osgbInteraction/DragHandler.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>
#include <osgwTools/NodePathUtils.h>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



//#define CREATE_SCENE
#ifdef CREATE_SCENE

#include <osg/MatrixTransform>
#include <osg/ProxyNode>
#include <osgDB/WriteFile>

void createScene()
{
    osg::Group* root = new osg::Group;
    root->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );

    osg::MatrixTransform* mt;
    osg::MatrixTransform* mt2;
    osg::MatrixTransform* mt3;
    osg::ProxyNode* pn;
    osg::Matrix m;

    m = osg::Matrix::translate( -12., 18., 6. );
    mt = new osg::MatrixTransform( m );
    root->addChild( mt );
    pn = new osg::ProxyNode();
    pn->setFileName( 0, "dumptruck.osg" );
    pn->setName( "dumptruck-body" );
    mt->addChild( pn );

    m = osg::Matrix::translate( 8., 5., 3.5 );
    mt = new osg::MatrixTransform( m );
    root->addChild( mt );
    m = osg::Matrix::scale( 3., 3., 3. );
    mt2 = new osg::MatrixTransform( m );
    mt->addChild( mt2 );
    pn = new osg::ProxyNode();
    pn->setFileName( 0, "tetra.osg" );
    pn->setName( "tetra-body" );
    mt2->addChild( pn );

    m = osg::Matrix::translate( 2., -6., 5. );
    mt = new osg::MatrixTransform( m );
    root->addChild( mt );
    m = osg::Matrix::scale( 4., 4., 4. );
    mt2 = new osg::MatrixTransform( m );
    mt->addChild( mt2 );
    m = osg::Matrix::rotate( -.4, 0., 0., 1. );
    mt3 = new osg::MatrixTransform( m );
    mt2->addChild( mt3 );
    pn = new osg::ProxyNode();
    pn->setFileName( 0, "dice.osg" );
    pn->setName( "dice-body" );
    mt3->addChild( pn );

    m = osg::Matrix::translate( -20., 8., 5. );
    mt = new osg::MatrixTransform( m );
    root->addChild( mt );
    m = osg::Matrix::rotate( 2.75, 0., 0., 1. );
    mt2 = new osg::MatrixTransform( m );
    mt->addChild( mt2 );
    pn = new osg::ProxyNode();
    pn->setFileName( 0, "cow.osg" );
    pn->setName( "cow-body" );
    mt2->addChild( pn );

    osgDB::writeNodeFile( *root, "saverestore-scene.osg" );
}

int main()
{
    createScene();
    return( 0 );
}

#else

btRigidBody* makeBody( osg::Node* node, const osg::Matrix& m )
{
    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    osgwTools::insertAbove( node, amt );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    cr->setCenterOfMass( node->getBound().center() );
    cr->_parentTransform = m;
    cr->_mass = 1.f;
    cr->_scale = m.getScale();
    cr->_restitution = .5f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Required for DragHandler default behavior.
    amt->setUserData( new osgbCollision::RefRigidBody( rb ) );

    return( rb );
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


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    std::string restoreFileName;
    if( arguments.read( "--restore", restoreFileName ) )
    {
        if( osgDB::findDataFile( restoreFileName ).empty() )
        {
            osg::notify( osg::FATAL ) << "Can't find restore file: \"" << restoreFileName << "\"." << std::endl;
            return( 1 );
        }
    }

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::ref_ptr< osg::Group > root = new osg::Group;

    const std::string sceneFileName( "saverestore-scene.osg" );
    osg::Node* scene = osgDB::readNodeFile( sceneFileName );
    if( scene == NULL )
    {
        osg::notify( osg::FATAL ) << "saverestore: Can't load data file \"" << sceneFileName << "\"." << std::endl;
        return( 1 );
    }
    root->addChild( scene );

    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new osgbInteraction::SaveRestoreHandler;

    // Restore the saved PhysicsState.
    if( !( restoreFileName.empty() ) )
        srh->restore( restoreFileName );

    // Find all nodes with names containing "-body". Turn each into a rigid body.
    osgwTools::FindNamedNode fnn( "-body" );
    fnn.setMatchMethod( osgwTools::FindNamedNode::CONTAINS );
    scene->accept( fnn );

    osgwTools::FindNamedNode::NodeAndPathList::const_iterator it;
    for( it = fnn._napl.begin(); it != fnn._napl.end(); it++ )
    {
        osg::Node* node = it->first;
        osg::NodePath np = it->second;
        osg::Matrix xform = osg::computeLocalToWorld( np );

        osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
        amt->setDataVariance( osg::Object::DYNAMIC );
        osgwTools::insertAbove( node, amt );

        // Manually insert the AMT above the node in the NodePath. Kind of ugly.
        np[ np.size() - 1 ] = amt;
        np.resize( np.size() + 1);
        np[ np.size() - 1 ] = node;

        const std::string npStr = osgwTools::nodePathToString( np );
        osg::ref_ptr< osgbDynamics::CreationRecord > cr;

        if( restoreFileName.empty() )
        {
            // Not restoring.
            cr = new osgbDynamics::CreationRecord;
            cr->_sceneGraph = amt;
            cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
            cr->setCenterOfMass( node->getBound().center() );
            cr->_parentTransform = xform;
            cr->_mass = 1.f;
            cr->_scale = xform.getScale();
            cr->_restitution = .5f;

            srh->add( npStr, cr );
        }
        else
        {
            // Restoring.
            osgbDynamics::PhysicsData* pd = srh->getPhysicsData( npStr );
            cr = pd->_cr;
        }

        btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );
        rb->setActivationState( DISABLE_DEACTIVATION );

        // Required for DragHandler default behavior.
        amt->setUserData( new osgbCollision::RefRigidBody( rb ) );

        if( restoreFileName.empty() )
            bulletWorld->addRigidBody( rb );

        srh->add( npStr, rb );
    }

    // Add ground
    const osg::Vec4 plane( 0., 0., 1., 0. );
    root->addChild( osgbDynamics::generateGroundPlane( plane, bulletWorld ) );


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
    //tb->setHomePosition( osg::Vec3( 0., -8., 2. ), osg::Vec3( 0., 0., 1. ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    srh->capture();
    viewer.addEventHandler( srh.get() );

    osgViewer::Viewer::Cameras cams;
    viewer.getCameras( cams );
    osg::ref_ptr< osgbInteraction::DragHandler > dh =
        new osgbInteraction::DragHandler( bulletWorld, cams[ 0 ] );
    viewer.addEventHandler( dh.get() );

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

#endif


/** \page saverestore Save and Restore Example

Demonstrates saving and restoring osgBullet data to/from disk.

Use the --debug command line option to enable debug collision object display.
*/
