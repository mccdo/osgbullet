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
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>
#include <osgbDynamics/TripleBuffer.h>
#include <osgbDynamics/PhysicsThread.h>

#include <btBulletDynamicsCommon.h>

#include <sstream>
#include <osg/io_utils>
#include <string>
#include <map>



osgbDynamics::TripleBuffer tBuf;
osgbDynamics::MotionStateList msl;


btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
    dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    return( dynamicsWorld );
}


/* \cond */
class InteractionManipulator : public osgGA::GUIEventHandler
{
public:
    InteractionManipulator( btDiscreteDynamicsWorld* world, osg::Group* sg, osgbDynamics::PhysicsThread* pt=NULL )
      : _world( world ),
        _sg( sg ),
        _pt( pt )
    {}

    void setInitialTransform( btRigidBody* rb, osg::Matrix m )
    {
        _posMap[ rb ] = m;
    }

    void updateView( osg::Camera* camera )
    {
        osg::Vec3 center, up;
        camera->getViewMatrixAsLookAt( _viewPos, center, up );
        _viewDir = center - _viewPos;
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
    {
        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::KEYUP:
            {
                if (ea.getKey()==osgGA::GUIEventAdapter::KEY_BackSpace)
                {
                    reset();
                    return true;
                }
                if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Return)
                {
                    fire();
                    return true;
                }

                return false;
            }

            default:
            break;
        }
        return false;
    }

protected:
    btDiscreteDynamicsWorld* _world;
    osg::ref_ptr< osg::Group > _sg;
    osgbDynamics::PhysicsThread* _pt;

    osg::Vec3 _viewPos, _viewDir;

    typedef std::map< btRigidBody*, osg::Matrix > PosMap;
    PosMap _posMap;

    typedef std::list< osg::ref_ptr< osg::Node > > NodeList;
    NodeList _nodeList;

    void reset()
    {
        PosMap::iterator it;
        for( it=_posMap.begin(); it!=_posMap.end(); it++ )
        {
            btRigidBody* rb = it->first;
            btTransform t = osgbCollision::asBtTransform( it->second );
            rb->setWorldTransform( t );
        }
    }

    void fire()
    {
        osg::Sphere* sp = new osg::Sphere( osg::Vec3( 0., 0., 0. ), .5 );
        osg::ShapeDrawable* shape = new osg::ShapeDrawable( sp );
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( shape );
        osg::ref_ptr< osgwTools::AbsoluteModelTransform > amt = new osgwTools::AbsoluteModelTransform;
        amt->addChild( geode );
        _sg->addChild( amt.get() );

        btSphereShape* collision = new btSphereShape( .5 );

        // Block physics
        if( _pt != NULL )
            // Blocks until thread is paused.
            _pt->pause( true );

        osgbDynamics::MotionState* motion = new osgbDynamics::MotionState;
        motion->setTransform( amt.get() );
        motion->setParentTransform( osg::Matrix::translate( _viewPos ) );

        btScalar mass( 0.2 );
        btVector3 inertia( btVector3( 0., 0., 0. ) );//osgbCollision::asBtVector3( _viewDir ) );
        collision->calculateLocalInertia( mass, inertia );
        btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
        btRigidBody* body = new btRigidBody( rbinfo );
        body->setLinearVelocity( osgbCollision::asBtVector3( _viewDir * 50. ) );
        _world->addRigidBody( body );

        // Set up for multithreading and triple buffering.
        motion->registerTripleBuffer( &tBuf );
        msl.push_back( motion );

        // Unblock physics
        if( _pt != NULL )
            _pt->pause( false );
    }
};
/* \endcond */


osg::ref_ptr< osg::Node > modelNode( NULL );

osg::Transform*
makeModel( const std::string& fileName, btDynamicsWorld* bw, osg::Vec3 pos, InteractionManipulator* im )
{
    osg::Matrix m( osg::Matrix::translate( pos ) );
    osg::ref_ptr< osgwTools::AbsoluteModelTransform > amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );

    if( !modelNode.valid() )
	{
        modelNode = osgDB::readNodeFile( fileName );
		if( !modelNode.valid() )
		{
			osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH is set correctly." << std::endl;
			exit( 0 );
		}
	}
    amt->addChild( modelNode.get() );


    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt.get();
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = .2;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

    osgbDynamics::MotionState* motion = static_cast< osgbDynamics::MotionState* >( rb->getMotionState() );
    motion->setParentTransform( m );
    rb->setWorldTransform( osgbCollision::asBtTransform( m ) );


    // Set up for multithreading and triple buffering.
    motion->registerTripleBuffer( &tBuf );
    msl.push_back( motion );

    im->setInitialTransform( rb, m );
    bw->addRigidBody( rb );

    return( amt.release() );
}

osg::MatrixTransform*
makeCow( btDynamicsWorld* bw, osg::Vec3 pos, InteractionManipulator* im )
{
    osg::Matrix m( osg::Matrix::rotate( 1.5, osg::Vec3( 0., 0., 1. ) ) *
        osg::Matrix::translate( pos ) );
    osg::MatrixTransform* root = new osg::MatrixTransform( m );
    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    root->addChild( amt );

	const std::string fileName( "cow.osg" );
    osg::Node* node = osgDB::readNodeFile( fileName );
	if( node == NULL )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
		exit( 0 );
	}
    amt->addChild( node );

    btCollisionShape* cs = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( node );
    osgbDynamics::MotionState* motion = new osgbDynamics::MotionState();
    motion->setTransform( amt );
    motion->setParentTransform( m );
    btScalar mass( 2. );
    btVector3 inertia( 0, 0, 0 );
    cs->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );

    // Set up for multithreading and triple buffering.
    motion->registerTripleBuffer( &tBuf );
    msl.push_back( motion );

    btRigidBody* body = new btRigidBody( rb );
    body->setActivationState( DISABLE_DEACTIVATION );
    im->setInitialTransform( body, m );
    bw->addRigidBody( body );

    return( root );
}


int
main( int argc,
      char ** argv )
{
    // Increase triple buffer size to hold lots of transform data.
    tBuf.resize( 16384 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osgbDynamics::PhysicsThread pt( bulletWorld, &tBuf );
    osg::Group* root = new osg::Group;

    InteractionManipulator* im = new InteractionManipulator( bulletWorld, root, &pt );

    std::string fileName( "dice.osg" );
    if( argc > 1 )
        // Seconf param is file name.
        fileName = std::string( argv[ 1 ] );

    // Make dice pyramid.
    int xCount( 7 );
    int yCount( 7 );
    float xStart( -4. );
    float yStart( -3. );
    const float zInc( 2.5 );
    float z( 1.75 );
    while( xCount && yCount )
    {
        float x, y;
        int xIdx, yIdx;
        for( y=yStart, yIdx=0; yIdx<yCount; y+=2.25, yIdx++ )
        {
            for( x=xStart, xIdx=0; xIdx<xCount; x+=2.25, xIdx++ )
            {
                osg::Vec3 pos( x, y, z );
                root->addChild( makeModel( fileName, bulletWorld, pos, im ) );
            }
        }
        xStart += 1.25;
        yStart += 1.25;
        xCount--;
        yCount--;
        z += zInc;
    }

    // Add a cow
    root->addChild( makeCow( bulletWorld, osg::Vec3( -11., 6., 4. ), im ) );

    // Make ground.
    {
        osg::Vec4 gp( 0, 0, 1, 0 );
        root->addChild( osgbDynamics::generateGroundPlane( gp, bulletWorld ) );
    }



    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setSceneData( root );
    viewer.addEventHandler( im );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -26., 12. ), osg::Vec3( 0., 0., 2. ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );

    viewer.addEventHandler( new osgViewer::StatsHandler );

    viewer.realize();
    pt.setProcessorAffinity( 0 );
    pt.start();

    while( !viewer.done() )
    {
        // Get the latest transform information from the
        // Bullet simulation.
        TripleBufferMotionStateUpdate( msl, &tBuf );

        viewer.frame();

        im->updateView( viewer.getCamera() );
    }

    pt.stopPhysics();
    pt.join();

    return( 0 );
}
