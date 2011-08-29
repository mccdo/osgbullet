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

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include <osgwTools/FindNamedNode.h>
#include <osgwTools/InsertRemove.h>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <iostream>


btDynamicsWorld * initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    return( dynamicsWorld );
}


/* \cond */
struct MyCallback : public osgbDynamics::MotionStateCallback
{
    unsigned int _count;

    MyCallback()
        : _count( 0 )
    {}

    virtual void operator()( const btTransform& worldTrans )
    {
        osg::notify( osg::ALWAYS ) << "Callback has been called " << ++_count << " times. Current x: " << worldTrans.getOrigin()[0] << std::endl;
    }
};
/* \endcond */


void
enablePhysics( osg::Node* root, const std::string& nodeName, btDynamicsWorld* bw )
{
    osgwTools::FindNamedNode fnn( nodeName );
    root->accept( fnn );
    if( fnn._napl.empty() )
    {
        osg::notify( osg::WARN ) << "Can't find node \"" << nodeName << "\"" << std::endl;
        return;
    }
    osg::Node* node = fnn._napl[ 0 ].first;
    osg::NodePath& np = fnn._napl[ 0 ].second;
    osg::BoundingSphere bs( node->getBound() );
    const osg::Matrix parentTrans = osg::computeLocalToWorld( np ); // Note that this might contain a scale.


    osg::ref_ptr< osgwTools::AbsoluteModelTransform > model( new osgwTools::AbsoluteModelTransform );
    model->setDataVariance( osg::Object::DYNAMIC );
    osgwTools::insertAbove( node, model.get() );

    btBoxShape* shape = osgbCollision::btBoxCollisionShapeFromOSG( model.get() );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = model.get();
    if( bs.center() != osg::Vec3( 0., 0., 0. ) )
        cr->setCenterOfMass( bs.center() );
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = 3.;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get(), shape );

    osgbDynamics::MotionState* motion = static_cast< osgbDynamics::MotionState* >( rb->getMotionState() );
    motion->getCallbackList().push_back( new MyCallback );
    motion->setParentTransform( parentTrans );

    rb->setAngularVelocity( btVector3( 3., 5., 0. ) );
    bw->addRigidBody( rb );
}


/* \cond */
class InteractionManipulator : public osgGA::GUIEventHandler
{
public:
    InteractionManipulator( btDynamicsWorld* world, osg::Group* sg )
      : _world( world ),
        _sg( sg )
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
    btDynamicsWorld* _world;
    osg::ref_ptr< osg::Group > _sg;

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

        osgbDynamics::MotionState* motion = new osgbDynamics::MotionState;
        motion->setTransform( amt.get() );

        motion->setParentTransform( osg::Matrix::translate( _viewPos ) );

        btScalar mass( 1. );
        btVector3 inertia( btVector3( 0., 0., 0. ) );//osgbCollision::asBtVector3( _viewDir ) );
        collision->calculateLocalInertia( mass, inertia );
        btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
        btRigidBody* body = new btRigidBody( rbinfo );
        body->setLinearVelocity( osgbCollision::asBtVector3( _viewDir * 50. ) );
        _world->addRigidBody( body );
    }
};
/* \endcond */


int main( int argc,
          char * argv[] )
{
    osg::ref_ptr< osg::Group > root = new osg::Group();
    btDynamicsWorld* bw = initPhysics();

    InteractionManipulator* im = new InteractionManipulator( bw, root.get() );

    root->addChild( osgbDynamics::generateGroundPlane( osg::Vec4( 0.f, 0.f, 1.f, -10.f ), bw ) );

    osg::MatrixTransform* mt( new osg::MatrixTransform( osg::Matrix::translate( 0., 0., 10. ) ) );
    root->addChild( mt );

	const std::string fileName( "block.osg" );
    osg::Node* block = osgDB::readNodeFile( fileName );
	if( block == NULL )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the osgBullet data directory." << std::endl;
		exit( 0 );
	}

	block->setName( "block" );
    mt->addChild( block );
    enablePhysics( root.get(), "block", bw );



    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.addEventHandler( im );
    osgGA::TrackballManipulator * tb = new osgGA::TrackballManipulator();
    tb->setHomePosition( osg::Vec3( 10, -55, 13 ),
                        osg::Vec3( 0, 0, -4 ),
                        osg::Vec3( 0, 0, 1 ) );
    viewer.setCameraManipulator( tb );
    viewer.setSceneData( root.get() );

    double currSimTime;
    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();

    viewer.realize();

    while( !viewer.done() )
    {
        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bw->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;
        viewer.frame();

        im->updateView( viewer.getCamera() );
    }

    return( 0 );
}

