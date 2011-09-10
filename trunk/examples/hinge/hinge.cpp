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

#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/FindNamedNode.h>
#include <osgwTools/GeometryOperation.h>
#include <osgwTools/GeometryModifier.h>

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btIDebugDraw.h>
#include <stdio.h> //printf debugging

#include <osg/io_utils>
#include <string>
#include <map>


//#define DO_DEBUG_DRAW
#ifdef DO_DEBUG_DRAW
#include <osgbCollision/GLDebugDrawer.h>
#endif


// Filter out collisions between the door and doorframe.
//
// Bullet collision filtering tutorial:
//   http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Collision_Filtering
//
// Define filter groups
enum CollisionTypes {
    COL_DOOR = 0x1 << 0,
    COL_DOORFRAME = 0x1 << 1,
    COL_DEFAULT = 0x1 << 2,
};
// Define filter masks
unsigned int doorCollidesWith( COL_DEFAULT );
unsigned int doorFrameCollidesWith( COL_DEFAULT );
unsigned int defaultCollidesWith( COL_DOOR | COL_DOORFRAME | COL_DEFAULT );


/** \cond */
class InteractionManipulator : public osgGA::GUIEventHandler
{
public:
    InteractionManipulator( btDiscreteDynamicsWorld* world, osg::Group* sg )
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
    btDiscreteDynamicsWorld* _world;
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
        float radius( .2 );
        const btVector3 velocity = osgbCollision::asBtVector3( _viewDir * 100. * radius );

        osg::Sphere* sp = new osg::Sphere( osg::Vec3( 0., 0., 0. ), radius );
        osg::ShapeDrawable* shape = new osg::ShapeDrawable( sp );
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( shape );
        osg::ref_ptr< osgwTools::AbsoluteModelTransform > amt = new osgwTools::AbsoluteModelTransform;
        amt->addChild( geode );
        _sg->addChild( amt.get() );

        btSphereShape* collision = new btSphereShape( radius );

        osgbDynamics::MotionState* motion = new osgbDynamics::MotionState;
        motion->setTransform( amt.get() );

        motion->setParentTransform( osg::Matrix::translate( _viewPos ) );

        btScalar mass( 0.2 );
        btVector3 inertia( btVector3( 0., 0., 0. ) );
        collision->calculateLocalInertia( mass, inertia );
        btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
        btRigidBody* body = new btRigidBody( rbinfo );
        body->setLinearVelocity( velocity );
        _world->addRigidBody( body, COL_DEFAULT, defaultCollidesWith );
    }
};


class FindGeomOp : public osgwTools::GeometryOperation
{
public:
    FindGeomOp() {}
    FindGeomOp( const FindGeomOp& rhs, const osg::CopyOp& copyOp=osg::CopyOp::SHALLOW_COPY ) {}
    META_Object(osgBulletExamples,FindGeomOp);

    virtual osg::Geometry* operator()( osg::Geometry& geom )
    {
        _target = &geom;
        return( &geom );
    }

    osg::ref_ptr< osg::Geometry > _target;
};
/** \endcond */


osg::Node* fixWalls( osg::Node* wallsNode )
{
    osg::ref_ptr< osg::Node > otherWall;
    {
        osg::ref_ptr< osg::Group > srcTempGroup = new osg::Group;
        srcTempGroup->addChild( wallsNode );
        osg::ref_ptr< osg::Group > otherWallTempGroup = new osg::Group( *srcTempGroup,
            osg::CopyOp::DEEP_COPY_NODES | osg::CopyOp::DEEP_COPY_DRAWABLES | osg::CopyOp::DEEP_COPY_PRIMITIVES );
        otherWall = otherWallTempGroup->getChild( 0 );
    }

    unsigned int count;
    {
        osg::ref_ptr< FindGeomOp > findGeom = new FindGeomOp;
        osgwTools::GeometryModifier modifier( findGeom.get() );
        wallsNode->accept( modifier );

        osg::Geometry* geom = findGeom->_target.get();
        osg::DrawArrays* da = dynamic_cast< osg::DrawArrays* >( geom->getPrimitiveSet( 0 ) );
        count = da->getCount();
        da->setCount( count / 2 );
    }
    {
        osg::ref_ptr< FindGeomOp > findGeom = new FindGeomOp;
        osgwTools::GeometryModifier modifier( findGeom.get() );
        otherWall->accept( modifier );

        osg::Geometry* geom = findGeom->_target.get();
        osg::DrawArrays* da = dynamic_cast< osg::DrawArrays* >( geom->getPrimitiveSet( 0 ) );
        da->setFirst( count / 2 );
        da->setCount( count / 2 );
    }

    return( otherWall.release() );
}


btRigidBody* doorFrame;

void makeDoorFrame( btDiscreteDynamicsWorld* bw, InteractionManipulator* im, osg::Node* node, const osg::Matrix& m )
{
    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform( m );
    osgwTools::insertAbove( node, amt );


    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_parentTransform = m;
    cr->_mass = 0.f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

    bw->addRigidBody( rb, COL_DOORFRAME, doorFrameCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, and also record its initial position in the InteractionManipulator (for reset)
    doorFrame = rb;
}

btRigidBody* door;
osg::Transform* makeDoor( btDiscreteDynamicsWorld* bw, InteractionManipulator* im, osg::Node* node, const osg::Matrix& m )
{
    // For COM.
    osg::BoundingSphere doorbb = node->getBound();

    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    osgwTools::insertAbove( node, amt );

    osg::Matrix invCOM = osg::Matrix::translate( -doorbb.center() );
    osg::ref_ptr< osg::MatrixTransform > mt = new osg::MatrixTransform( invCOM );
    mt->addChild( node );
    btCollisionShape* shape = osgbCollision::btCompoundShapeFromBounds( mt.get(), BOX_SHAPE_PROXYTYPE );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->setCenterOfMass( doorbb.center() );
    cr->_parentTransform = m;
    cr->_mass = 1.f;
    cr->_restitution = .5f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get(), shape );

    btTransform wt;
    rb->getMotionState()->getWorldTransform( wt );
    rb->setWorldTransform( wt );


    bw->addRigidBody( rb, COL_DOOR, doorCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, and also record its initial position in the InteractionManipulator (for reset)
    door = rb;
    im->setInitialTransform( rb, m );

    return( amt );
}

btDiscreteDynamicsWorld*
initPhysics()
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
    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::Group* root = new osg::Group;

    InteractionManipulator* im = new InteractionManipulator( bulletWorld, root );



    osg::ref_ptr< osg::Node > rootModel = osgDB::readNodeFile( "GateWall.flt" );
    if( !rootModel.valid() )
    {
        osg::notify( osg::FATAL ) << "hinge: Can't load data file \"GateWall.flt\"." << std::endl;
        return( 1 );
    }

    root->addChild( rootModel.get() );

    // As far as our code is concerned, the walls and the wall hinges are just
    // Bullet static objects. But the model is set up in such a way that their
    // information must be extracted separately.

    // Get Node pointers and parent transforms for the walls, wall hinges, and
    // the gate. (Node names are taken from the osgWorks osgwnames utility.)
    osg::Matrix wallXform, hingeXform, gateXform;
    osg::Node* wallsNode = findNamedNode( rootModel.get(), "Walls", wallXform );
    osg::Node* hingesNode = findNamedNode( rootModel.get(), "WallHinges", hingeXform );
    osg::Node* gateNode = findNamedNode( rootModel.get(), "DOF_Gate", gateXform );
    if( ( wallsNode == NULL ) || ( hingesNode == NULL ) || ( gateNode == NULL ) )
        return( 1 );

    // Unfortunately, the walls come to us not just as a single Geode, but also as
    // a single Geometry and single PrimitiveSet. Break that into two Geodes, a
    // left wall and a right wall, so we can put a box collision shape around each.
    osg::ref_ptr< osg::Node > leftWall, rightWall;
    osg::Node* otherWall = fixWalls( wallsNode );
    wallsNode->getParent( 0 )->addChild( otherWall );
    otherWall->setName( "otherWall" );
    osg::Matrix otherWallXform = wallXform;

    // Add door
    osg::Transform* doorAMT = makeDoor( bulletWorld, im, gateNode, gateXform );

    // Add doorframe
    makeDoorFrame( bulletWorld, im, wallsNode, wallXform );
    makeDoorFrame( bulletWorld, im, otherWall, otherWallXform );
    makeDoorFrame( bulletWorld, im, hingesNode, hingeXform );


    // Make ground
    if( 1 )
    {
        const double planeDistance( 0. );
        osg::Geode* geode = new osg::Geode;
        geode->addDrawable( osgwTools::makePlane( osg::Vec3( -10., -10., planeDistance ),
            osg::Vec3( 20., 0., 0. ), osg::Vec3( 0., 20., 0. ) ) );
        root->addChild( geode );
        btCollisionShape* groundShape = new btStaticPlaneShape( btVector3( 0, 0, 1 ), planeDistance );
        btRigidBody::btRigidBodyConstructionInfo rbInfo( 0., NULL, groundShape, btVector3(0,0,0) );
        btRigidBody* ground = new btRigidBody(rbInfo);
        bulletWorld->addRigidBody( ground );
    }



    {
        // Pivot point and pivot axis are both in the door's object space.
        // Note that the door is COM-adjusted, so the pivot point must also be
        // in the door's COM-adjusted object space.
        // TBD extract this from hinge data fine.
        const btVector3 btPivot( -0.498f, -0.019f, 0.146f );

        btVector3 btAxisA( 0., 0., 1. );
        btHingeConstraint* hinge = new btHingeConstraint( *door, btPivot, btAxisA );
        hinge->setLimit( -1.5f, 1.5f );
        bulletWorld->addConstraint( hinge, true );
    }


#ifdef DO_DEBUG_DRAW
    osgbCollision::GLDebugDrawer* dbgDraw = new osgbCollision::GLDebugDrawer();
    dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
    bulletWorld->setDebugDrawer( dbgDraw );
    root->addChild( dbgDraw->getSceneGraph() );
#endif


    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root );
    viewer.addEventHandler( im );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -26., 12. ), osg::Vec3( 0., 0., 2. ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
#ifdef DO_DEBUG_DRAW
        dbgDraw->BeginDraw();
#endif

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

#ifdef DO_DEBUG_DRAW
        bulletWorld->debugDrawWorld();
        dbgDraw->EndDraw();
#endif

        viewer.frame();

        im->updateView( viewer.getCamera() );
    }

    return( 0 );
}


/** \page hingelowlevel Simple Hinge Constraint

Example description TBD.
*/
