// Copyright (c) 2009 Skew Matrix Software LLC. All rights reserved.

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>

#include <osgbBullet/OSGToCollada.h>
#include <osgbBullet/ColladaUtils.h>
#include <osgbBullet/MotionState.h>
#include <osgwTools/AbsoluteModelTransform.h>
#include <osgbBullet/Utils.h>

#include <btBulletDynamicsCommon.h>

#include <stdlib.h>
#include <string>
#include <osg/io_utils>


btDiscreteDynamicsWorld*
initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

    return( dynamicsWorld );
}


void
testCase0Util( const std::string& iveName, const std::string& daeName, btDiscreteDynamicsWorld* bw )
{
    osg::ref_ptr< osg::Node > node0 = osgDB::readNodeFile( iveName );
    if( !node0.valid() )
    {
        osg::notify( osg::FATAL ) << "Can't load .ive: " << iveName << std::endl;
        return;
    }
    osg::ref_ptr< osg::MatrixTransform > mt0 = new osg::MatrixTransform;
    mt0->setDataVariance( osg::Object::DYNAMIC );
    mt0->addChild( node0.get() );

    osg::NodePath np;
    np.push_back( new osg::MatrixTransform );
    if( !( osgbBullet::loadDae( mt0.get(), np, daeName, bw ) ) )
    {
        osg::notify( osg::FATAL ) << "Can't load .dae: " << daeName << std::endl;
        return;
    }
}

void
testCase0( btDiscreteDynamicsWorld* bw )
{
    testCase0Util( "USMC23_4009.ASM.IVE", "USMC23_4009.ASM2.dae", bw );
    testCase0Util( "USMC23_4011.ASM.IVE", "USMC23_4011.ASM2.dae", bw );
    testCase0Util( "USMC23_4019.ASM.IVE", "USMC23_4019.ASM2.dae", bw );
}

void
testCase1and2Support( osg::Transform* xform, btDiscreteDynamicsWorld* bw )
{
    osg::Node* node = xform->getChild( 0 );
    osg::Group* parent = xform;
    osg::NodePath np;
    np.push_back( xform );
    const osg::Matrix parentTrans( osg::Matrix::identity() );

    // Copy the subgraph for use with OSGToCollada.
    osg::Group* asGrp = node->asGroup();
    osg::ref_ptr< osg::Group > copyGrp = new osg::Group( *asGrp, osg::CopyOp::DEEP_COPY_ALL );

    osgbBullet::OSGToCollada converter;
    converter.setSceneGraph( copyGrp.get() );
    converter.setShapeType( BOX_SHAPE_PROXYTYPE );
    converter.setMass( 1. );
    converter.setOverall( false );

    converter.convert();

    osg::ref_ptr< osgwTools::AbsoluteModelTransform > model( new osgwTools::AbsoluteModelTransform );
    model->setDataVariance( osg::Object::DYNAMIC );
    model->addChild( node );

    btRigidBody* rb = converter.getRigidBody();
    osgbBullet::MotionState* motion = new osgbBullet::MotionState;
    motion->setTransform( model.get() );
    motion->setCenterOfMass( converter.getCenterOfMass() );
    motion->setParentTransform( parentTrans );
    rb->setMotionState( motion );
    bw->addRigidBody( rb );

    parent->addChild( model.get() );
    parent->removeChild( node );
}

void
testCase1Util( const std::string& iveName, btDiscreteDynamicsWorld* bw )
{
    osg::ref_ptr< osg::Node > node0 = osgDB::readNodeFile( iveName );
    if( !node0.valid() )
    {
        osg::notify( osg::FATAL ) << "Can't load .ive: " << iveName << std::endl;
        return;
    }
    osg::ref_ptr< osg::MatrixTransform > mt0 = new osg::MatrixTransform;
    mt0->setDataVariance( osg::Object::DYNAMIC );
    mt0->addChild( node0.get() );

    testCase1and2Support( mt0.get(), bw );
}

void
testCase1( btDiscreteDynamicsWorld* bw )
{
    testCase1Util( "USMC23_4009.ASM.IVE", bw );
    testCase1Util( "USMC23_4011.ASM.IVE", bw );
    testCase1Util( "USMC23_4019.ASM.IVE", bw );
}

void
testCase2Util( const std::string& iveName, btDiscreteDynamicsWorld* bw )
{
    osg::ref_ptr< osg::Node > node0 = osgDB::readNodeFile( iveName );
    if( !node0.valid() )
    {
        osg::notify( osg::FATAL ) << "Can't load .ive: " << iveName << std::endl;
        return;
    }
    osg::ref_ptr< osg::MatrixTransform > mt0 = new osg::MatrixTransform;
    mt0->setDataVariance( osg::Object::DYNAMIC );
    mt0->addChild( node0.get() );

    // TBD run reducer.

    testCase1and2Support( mt0.get(), bw );
}

void
testCase2( btDiscreteDynamicsWorld* bw )
{
    testCase2Util( "USMC23_4009.ASM.IVE", bw );
    testCase2Util( "USMC23_4011.ASM.IVE", bw );
    testCase2Util( "USMC23_4019.ASM.IVE", bw );
}


int
main( int argc,
      char ** argv )
{
    if( argc != 2)
    {
        osg::notify( osg::FATAL ) << "Please specify command line argument:" << std::endl;
        osg::notify( osg::FATAL ) << "\t0 - Time loading physics data from .dae." << std::endl;
        osg::notify( osg::FATAL ) << "\t1 - Time creating physics data at runtime." << std::endl;
        osg::notify( osg::FATAL ) << "\t2 - Time decimation and tuntime physics creation." << std::endl;
        return( 1 );
    }

    btDiscreteDynamicsWorld* bw = initPhysics();
    const int test( atoi( argv[ 1 ] ) );

    osg::Timer timer;

    switch( test )
    {
    case 0:
    {
        testCase0( bw );
        break;
    }
    case 1:
    {
        testCase1( bw );
        break;
    }
    case 2:
    {
        testCase2( bw );
        break;
    }
    default:
    {
        break;
    }
    }

    osg::notify( osg::ALWAYS ) << "Elapsed time for test " << test << std::endl;
    osg::notify( osg::ALWAYS ) << timer.time_s() << std::endl;

    return( 0 );
}

