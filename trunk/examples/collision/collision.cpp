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

#include <btBulletCollisionCommon.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventHandler>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osgwTools/Shapes.h>
#include <osgwTools/Version.h>

#include <osg/io_utils>
#include <iostream>



class MoveManipulator : public osgGA::GUIEventHandler
{
public:
    MoveManipulator() : _co( NULL ) {}
    MoveManipulator( const MoveManipulator& mm, osg::CopyOp copyop ) : _co( mm._co ) {}
    ~MoveManipulator() {}
#if( OSGWORKS_OSG_VERSION > 20800 )
    META_Object(osgBulletExample,MoveManipulator);
#endif

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        //if( ea.getEventType() == osgGA::GUIEventAdapter::DRAG
        return( false );
    }

    void setCollisionObject( btCollisionObject* co ) { _co = co; }

protected:
    btCollisionObject* _co;
};


btCollisionWorld* initCollision()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btCollisionWorld* collisionWorld = new btCollisionWorld( dispatcher, inter, collisionConfiguration );

    return( collisionWorld );
}



osg::Node* createScene( btCollisionWorld* cw, MoveManipulator* mm, osg::ArgumentParser& arguments )
{
    osg::ref_ptr< osg::Group > root = new osg::Group;

    // Create a static box
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );
    root->addChild( geode );

    btCollisionObject* btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( osgbCollision::btBoxCollisionShapeFromOSG( geode ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
    cw->addCollisionObject( btBoxObject );


    // Create a box we can drag around with the mouse
    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );
    osg::Matrix transMatrix = osg::Matrix::translate( 4., 0., 0. );
    osg::MatrixTransform* mt = new osg::MatrixTransform( transMatrix );
    mt->addChild( geode );
    root->addChild( mt );

    btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( osgbCollision::btBoxCollisionShapeFromOSG( geode ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    btBoxObject->setWorldTransform( osgbCollision::asBtTransform( transMatrix ) );
    cw->addCollisionObject( btBoxObject );
    mm->setCollisionObject( btBoxObject );


    return( root.release() );
}

int main( int argc,
          char * argv[] )
{
    btCollisionWorld* collisionWorld = initCollision();

    osg::ArgumentParser arguments( &argc, argv );
    MoveManipulator* mm = new MoveManipulator;
    osg::ref_ptr< osg::Node > root = createScene( collisionWorld, mm, arguments );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator() );
    viewer.addEventHandler( mm );
    viewer.setSceneData( root.get() );


    while( !viewer.done() )
    {
        collisionWorld->performDiscreteCollisionDetection();
        osg::notify( osg::ALWAYS ) <<
            collisionWorld->getDispatcher()->getNumManifolds() << std::endl;

        viewer.frame();
    }

    return( 0 );
}
