/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009 by Kenneth Mark Bryden
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
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <btBulletDynamicsCommon.h>


#include <osg/io_utils>
#include <iostream>

#include <osgbBullet/MotionState.h>
#include <osgbBullet/CollisionShapes.h>
#include <osgbBullet/RefRigidBody.h>
#include <osgwTools/AbsoluteModelTransform.h>
#include <osgbBullet/OSGToCollada.h>
#include <osgbBullet/ColladaUtils.h>
#include <osgbBullet/Utils.h>



btDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ));

    return( dynamicsWorld );
}


class DaeLoader : public osgGA::GUIEventHandler
{
public:
    DaeLoader( osg::Transform* n, const std::string& baseName, btDynamicsWorld* dw )
      : _n( n ),
        _baseName( baseName ),
        _dw( dw )
    {
    }
    ~DaeLoader()
    {
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
    {
        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::KEYUP:
            {
                int key( ea.getKey() );
                if( (key<'0') || (key>'9') )
                    return false;

                osg::Matrix initialTrans;
                osgwTools::AbsoluteModelTransform* amt = dynamic_cast< osgwTools::AbsoluteModelTransform* >( _n.get() );
                if( amt != NULL ) {
                    initialTrans = amt->getMatrix();
                    amt->setMatrix( osg::Matrix::identity() );
                }

                osgbBullet::RefRigidBody* rb = dynamic_cast< osgbBullet::RefRigidBody* >( _n->getUserData() );
                if( rb != NULL )
                {
                    osg::notify( osg::ALWAYS ) << "*** Removing rigid body" << std::endl;
                    btRigidBody* brb = rb->getRigidBody();
                    osgbBullet::MotionState* motion = dynamic_cast< osgbBullet::MotionState* >( brb->getMotionState() );
                    _dw->removeRigidBody( brb );
                }

                char keych[2] = { key, 0 };
                std::string baseName( _baseName + std::string( (char*)&keych ) );
                std::string daeName( baseName + ".dae" );

                osg::NodePath np;
                np.push_back( new osg::MatrixTransform( initialTrans ) );
                if( !( osgbBullet::loadDae( _n.get(), np, daeName, _dw ) ) )
                    return false;

                return true;
            }
            default:
            break;
        }
        return false;
    }

protected:
    osg::ref_ptr< osg::Transform > _n;
    std::string _baseName;
    btDynamicsWorld* _dw;

};



osg::MatrixTransform* createOSGBox( osg::Vec3 size )
{
    osg::Box* box = new osg::Box();
    box->setHalfLengths( size );

    osg::ShapeDrawable* shape = new osg::ShapeDrawable( box );
    shape->setColor( osg::Vec4( 1., 1., 1., 1. ) );
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::MatrixTransform* mt = new osg::MatrixTransform();
    mt->addChild( geode );

    return( mt );
}

osg::Node*
createGround( float w, float h, const osg::Vec3& center )
{
    osg::MatrixTransform* ground = createOSGBox( osg::Vec3( w, h, .1 ));

    osgbBullet::OSGToCollada converter;
    converter.setSceneGraph( ground );
    converter.setShapeType( BOX_SHAPE_PROXYTYPE );
    converter.setMass( 0.f );
    converter.convert();

    btRigidBody* body = converter.getRigidBody();

    // OSGToCollada counts on FLATTEN_STATIC_TRANSFORMS to transform all
    // verts, but that doesn't work with ShapeDrawables, so we must
    // transform the box explicitly.
    btTransform transform; transform.setIdentity();
    transform.setOrigin( osgbBullet::asBtVector3( center ) );
    body->getMotionState()->setWorldTransform( transform );
    body->setWorldTransform( transform );

    ground->setUserData( new osgbBullet::RefRigidBody( body ) );

    return ground;
}

int main( int argc,
          char* argv[] )
{
    if( argc < 2 )
    {
        osg::notify( osg::FATAL ) << "Usage: colladaread <filename>" << std::endl;
        return 1;
    }
    std::string fileName( argv[ 1 ] );

    osg::ref_ptr< osg::Node > load = osgDB::readNodeFile( fileName );
    if( !load )
    {
        osg::notify( osg::FATAL ) << "Can't load input file(s)." << std::endl;
        return 1;
    }
    std::string baseName( osgDB::getNameLessExtension( fileName ) );
    osg::notify( osg::ALWAYS ) << "colladaread: Loaded model from " << fileName << std::endl;
    osg::notify( osg::ALWAYS ) << "Press the '0' key to load " << baseName << "0.dae (for example)." << std::endl;

    osg::ref_ptr< osgwTools::AbsoluteModelTransform > model =
        new osgwTools::AbsoluteModelTransform;
    model->addChild( load.get() );



    btDynamicsWorld* dynamicsWorld = initPhysics();

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator();
    viewer.setCameraManipulator( tb );
    viewer.addEventHandler( new DaeLoader( model.get(), baseName, dynamicsWorld ) );

    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild( model.get() );


    // Compute a reasonable ground plane size based on the bounding sphere radius.
    float dim = load->getBound()._radius * 1.5;
    osg::Vec3 cen = load->getBound()._center;
    cen[ 2 ] -= dim;
    osg::ref_ptr< osg::Node > ground = createGround( dim, dim, cen );
    root->addChild( ground.get() );
    osgbBullet::RefRigidBody* body = dynamic_cast< osgbBullet::RefRigidBody* >( ground->getUserData() );
    dynamicsWorld->addRigidBody( body->getRigidBody() );


    double currSimTime;
    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();

    viewer.setSceneData( root.get() );
    viewer.realize();

    while( !viewer.done())
    {
        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;
        viewer.frame();
    }

    return( 0 );
}
