// Copyright (c) 2009 Skew Matrix Software LLC. All rights reserved.

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Geometry>

#include <osgwTools/AbsoluteModelTransform.h>
#include <osgwTools/Grid.h>

#include <osgbBullet/OSGToCollada.h>
#include <osgbBullet/MotionState.h>
#include <osgbBullet/Utils.h>
#include <osgbBullet/GLDebugDrawer.h>

#include <btBulletDynamicsCommon.h>

#include <osgwTools/FindNamedNode.h>

#include <vector>
#include <string>
#include <osg/io_utils>


const bool showOrig( true );
const bool showCube( false );
const bool showTruck( false );
const bool enableOrig( showOrig && true );
const bool enableCube( showCube && true );
const bool enableTruck( showTruck && true );

void enablePhysics( osg::Node* root, const std::string& nodeName, btDynamicsWorld* bw, const osg::Vec3& com=osg::Vec3( 0., 0., 0. ) );



osg::Node*
makeScene( btDynamicsWorld* bw )
{
    osg::Switch* root = new osg::Switch;


    // Make the ground plane
    {
        btBoxShape* shape = new btBoxShape( btVector3( 40., 40., .5 ) );

		btRigidBody::btRigidBodyConstructionInfo rbInfo( 0., NULL, shape );
		btRigidBody* body = new btRigidBody( rbInfo );
        body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
        body->setActivationState( DISABLE_DEACTIVATION );

		bw->addRigidBody(body);
	}

    // Make wireframe reference grids to aid in visual verification of location.
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeGrid( osg::Vec3( -10, -10, 10 ), osg::Vec3( 20, 0, 0 ), osg::Vec3( 0, 20, 0 ), 5 ) );
    geode->addDrawable( osgwTools::makeGrid( osg::Vec3( -10, 0, 0 ), osg::Vec3( 20, 0, 0 ), osg::Vec3( 0, 0, 20 ), 5 ) );
    root->addChild( geode );


    osg::ref_ptr< osg::Node > cowNode = osgDB::readNodeFile( "cow.osg" );
    osg::ref_ptr< osg::Node > truckNode = osgDB::readNodeFile( "dumptruck.osg" );
    osg::ref_ptr< osg::Node > cubeNode = osgDB::readNodeFile( "offcube.osg" );
    osg::ref_ptr< osg::Node > blockNode = osgDB::readNodeFile( "block.osg" );


    osg::Group* origGrp = new osg::Group;

    // Cow, not scaled, off-origin COM.
    osg::MatrixTransform* mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -7, 7, 13 ) ) );
    osg::MatrixTransform* mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( 2., osg::Vec3( 0, 1, 0 ) ) );
    osg::MatrixTransform* mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( 1., 1., 1. ) ) );
    osg::Group* grp = new osg::Group;
    grp->setName( "cow0" );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cowNode.get() );

    // Truck, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 7, 7, 13 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .9, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .3, .3, .3 ) ) );
    grp = new osg::Group;
    grp->setName( "truck0" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 0, 8, 9 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( -.6, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( 3.5, 3.5, 3.5 ) ) );
    grp = new osg::Group;
    grp->setName( "offcube0" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Truck, global scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .25, .25, .25 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -30, -21, 39 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::rotate( -.9, osg::Vec3( 0, 1, 0 ) ) );
    grp = new osg::Group;
    grp->setName( "truck1" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    // Block, local scale, origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 9, -7, 13 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .6, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .5, .5, .5 ) ) );
    grp = new osg::Group;
    grp->setName( "block0" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( blockNode.get() );

    // Block test case
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 5, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( 0., osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .5, .5, .5 ) ) );
    grp = new osg::Group;
    grp->setName( "block1" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( blockNode.get() );

    const unsigned int origGrpIdx( root->getNumChildren() );
    root->addChild( origGrp );


    osg::Group* cubeGrp = new osg::Group;
    const float cubeScale( 2.f );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -10, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .1, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube1" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -5, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .2, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube2" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 0, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .3, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube3" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 5, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .4, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube4" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 10, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .5, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube5" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    const unsigned int cubeGrpIdx( root->getNumChildren() );
    root->addChild( cubeGrp );


    osg::Group* truckGrp = new osg::Group;
    float sFac( .25 );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -18, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI/2., osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck2" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -9, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck3" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 0, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI*1.5, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck4" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 9, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI*2., osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck5" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    const unsigned int truckGrpIdx( root->getNumChildren() );
    root->addChild( truckGrp );


    root->setValue( origGrpIdx, showOrig );
    root->setValue( cubeGrpIdx, showCube );
    root->setValue( truckGrpIdx, showTruck );

    if( enableOrig )
    {
        enablePhysics( root, "cow0", bw );
        enablePhysics( root, "truck0", bw );
        enablePhysics( root, "offcube0", bw );
        enablePhysics( root, "truck1", bw );
        enablePhysics( root, "block0", bw );
        enablePhysics( root, "block1", bw, osg::Vec3( 3., 0., 0. ) );
    }

    if( enableCube )
    {
        enablePhysics( root, "offcube1", bw );
        enablePhysics( root, "offcube2", bw );
        enablePhysics( root, "offcube3", bw );
        enablePhysics( root, "offcube4", bw );
        enablePhysics( root, "offcube5", bw );
    }

    if( enableTruck )
    {
        enablePhysics( root, "truck2", bw );
        enablePhysics( root, "truck3", bw );
        enablePhysics( root, "truck4", bw );
        enablePhysics( root, "truck5", bw );
    }


    return( (osg::Node*) root );
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


osg::Vec3
findScale( osg::NodePath& np )
{
    // Note this code is doing a top-down search rather than a bottom up search.
    // Really this is of no consequence; either way, the code stops as soon as it
    // finds a scale, so it only supports a single scale in the NodePath and can
    // therefore search in either direction.
    osg::NodePath::const_iterator it;
    for( it=np.begin(); it != np.end(); it++ )
    {
        osg::Node* node( *it );
        osg::Transform* trans( node->asTransform() );
        if( trans == NULL )
            continue;
        osg::MatrixTransform* mt = dynamic_cast< osg::MatrixTransform* >( trans );
        osg::PositionAttitudeTransform* pat = dynamic_cast< osg::PositionAttitudeTransform* >( trans );
        if( mt != NULL )
        {
            const osg::Matrix& m( mt->getMatrix() );
            osg::Vec3 sVec = m.getScale();
            if( sVec != osg::Vec3( 1., 1., 1. ) )
                return( sVec );
        }
        else if( pat != NULL )
        {
            osg::Vec3 sVec = pat->getScale();
            if( sVec != osg::Vec3( 1., 1., 1. ) )
                return( sVec );
        }
    }
    return( osg::Vec3( 1., 1., 1. ) );
}

// Find the Node with name nodeName in the scene graph rooted at root.
// Enable physics for that node using the bullet world bw. If the COM
// is anything other than (0,0,0), use it.
void
enablePhysics( osg::Node* root, const std::string& nodeName, btDynamicsWorld* bw, const osg::Vec3& com )
{
    bool useCom( com != osg::Vec3( 0., 0., 0. ) );

    osgwTools::FindNamedNode fnn( nodeName );
    root->accept( fnn );
    if( fnn._napl.empty() )
    {
        osg::notify( osg::WARN ) << "Can't find node \"" << nodeName << "\"" << std::endl;
        return;
    }
    osg::Node* node = fnn._napl[ 0 ].first;
    osg::BoundingSphere bs( node->getBound() );
    osg::Group* parent = node->getParent( 0 );
    osg::NodePath np = fnn._napl[ 0 ].second;
    const osg::Matrix parentTrans = osg::computeLocalToWorld( np ); // Note that this might contain a scale.

    // Find out if we are scaled, and if so, what is the xyz scale vector.
    const osg::Vec3 sVec = findScale( np );
    const bool scaled( sVec != osg::Vec3( 1., 1., 1. ) );


    // Copy the subgraph for use with OSGToCollada.
    osg::Group* asGrp = node->asGroup();
    osg::ref_ptr< osg::Group > copyGrp = new osg::Group( *asGrp, osg::CopyOp::DEEP_COPY_ALL );

    osgbBullet::OSGToCollada converter;
#if 0
    // This currently crashes and has other issues. Many operations in
    // OSGToCOllada don't adhere to this setting, e.h., auto compute COM
    // is done on the _whole_ sg, FlattenStatisTransforms is done on the
    // _whole_ sg, etc.
    converter.setSceneGraph( root );
    converter.setNodeName( nodeName );
#else
    converter.setSceneGraph( copyGrp.get() );
#endif
    converter.setShapeType( CONVEX_HULL_SHAPE_PROXYTYPE );
    converter.setMass( 1. );
    converter.setOverall( true );
    // Note that if the scale transform is above the subgraph we pass in to OSGToCollada,
    // we must explicitly pass the scale in, so that OSGToCollada will scale the
    // collition shape.
    if( scaled )
        converter.setScale( sVec );
    // OSGToCollada needs to know about an explicit COM. Passing in COM disables auto
    // compute of COM, which would use the bounding sphere center as the COM.
    if( useCom )
        converter.setCenterOfMass( com );

    converter.convert();

    osg::ref_ptr< osgwTools::AbsoluteModelTransform > model( new osgwTools::AbsoluteModelTransform );
    model->setDataVariance( osg::Object::DYNAMIC );
    model->addChild( node );

    btRigidBody* rb = converter.getRigidBody();
    osgbBullet::MotionState* motion = new osgbBullet::MotionState;
    motion->setTransform( model.get() );
    // To position the collision shape and the OSG visual rep correctly, MotionState needs
    // three pieces of data: the COM, the scaling factor, and the parent transform (or
    // initial l2w transform).
    if( useCom )
        motion->setCenterOfMass( com );
    else if( bs.center() != osg::Vec3( 0., 0., 0. ) )
        // If we don't have an explicit COM, and OSGToCollada auto compute was enabled (default),
        // then we need to explicitly set the COM here to be the bounding sphere center.
        motion->setCenterOfMass( bs.center() );
    // The scaling factor will either be (1,1,1) or the scale that we found in the NodePath.
    // Either way, just go ahead and set it here, though MotionState defaults the scale to
    // (1,1,1) so we really don't need to set it in that case.
    motion->setScale( sVec );
    // The parent transform (the local to world matrix extracted from the NodePath) might contain
    // a scale. However, the setParentTransform function orthonormalizes the matrix, which
    // eliminates any scale. So, even though Bullet doesn't support scaling, you can still pass in
    // a parent transform with a scale, and MotionState will just ignore it. (You must pass in any
    // parent scale transform using setScale.)
    motion->setParentTransform( parentTrans );
    rb->setMotionState( motion );
    bw->addRigidBody( rb );

    parent->addChild( model.get() );
    parent->removeChild( node );
}


int
main( int argc,
      char ** argv )
{
    btDynamicsWorld* bulletWorld = initPhysics();
    osg::ref_ptr< osg::Group > root = new osg::Group;

    root->addChild( makeScene( bulletWorld ) );


    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setSceneData( root.get() );
    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -70., 12. ), osg::Vec3( 0., 0., 12. ), osg::Vec3( 0., 0., 1. ) );
    viewer.setCameraManipulator( tb );


    osgbBullet::GLDebugDrawer* dbgDraw = new osgbBullet::GLDebugDrawer();
    dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
    bulletWorld->setDebugDrawer( dbgDraw );
    root->addChild( dbgDraw->getSceneGraph() );

    double currSimTime = viewer.getFrameStamp()->getSimulationTime();
    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();
    viewer.realize();

    while( !viewer.done() )
    {
        dbgDraw->BeginDraw();

        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        bulletWorld->debugDrawWorld();
        dbgDraw->EndDraw();

        viewer.frame();
    }

    return( 0 );
}

