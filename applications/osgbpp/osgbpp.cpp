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

#include <osgbBullet/Version.h>
#include <osgbBullet/MotionState.h>
#include <osgbBullet/CollisionShapes.h>
#include <osgbBullet/RefRigidBody.h>
#include <osgbBullet/OSGToCollada.h>
#include <osgbBullet/Utils.h>

#include <osgwTools/AbsoluteModelTransform.h>

#include <osg/io_utils>
#include <iostream>
#include <sstream>



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

osg::Transform* createOSGBox( osg::Vec3 size )
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
    osg::Transform* ground = createOSGBox( osg::Vec3( w, h, .01 ) );

    osgbBullet::OSGToCollada converter;
    converter.setSceneGraph( ground );
    converter.setShapeType( BOX_SHAPE_PROXYTYPE );
    converter.setMass( 0.f );
    converter.convert();

    btRigidBody* body = converter.getRigidBody();

    // OSGToCollada flattens transformation to transform all
    // verts, but that doesn't work with ShapeDrawables, so we must
    // transform the box explicitly.
    osgbBullet::MotionState* motion = dynamic_cast< osgbBullet::MotionState* >( body->getMotionState() );
    osg::Matrix m( osg::Matrix::translate( center ) );
    motion->setParentTransform( m );
    body->setWorldTransform( osgbBullet::asBtTransform( m ) );

    ground->setUserData( new osgbBullet::RefRigidBody( body ) );

    return ground;
}

int main( int argc,
          char* argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " creates physics data for model files and stores that data to COLLADA files." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "--com <x>,<y>,<z>", "Specifies the center of mass. If not specifies, osgbpp uses the center of the OSG bounding sphere." );
    arguments.getApplicationUsage()->addCommandLineOption( "--box", "This is the default. Creates a box collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--sphere", "Creates a sphere collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--cylinder", "Creates a cylinder collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--axis <x>", "This argument is ignored if --cylinder is not specified. Use this option to specify the cylinder axis X, Y, or Z. Default is Z." );
    arguments.getApplicationUsage()->addCommandLineOption( "--triMesh", "It creates a tri mesh collision shape (suitable for static objects)." );
    arguments.getApplicationUsage()->addCommandLineOption( "--convexTM", "Creates a convex tri mesh collision shape." );
    arguments.getApplicationUsage()->addCommandLineOption( "--convexHull", "Creates a convex hull collision shape." );

    arguments.getApplicationUsage()->addCommandLineOption( "--overall", "Creates a single collision shape for the entire input scene graph or named subgraph (see --name), rather than a collision shape per Geode, which is the default." );
    arguments.getApplicationUsage()->addCommandLineOption( "--name <name>", "Interprets the scene graph from the first occurence of the named node. If not specified, the entire scene graph is processed." );
    arguments.getApplicationUsage()->addCommandLineOption( "--mass <n>", "Specifies the desired rigid body mass value. The default is 1.0." );
    arguments.getApplicationUsage()->addCommandLineOption( "-o <name.dae>", "Output file name. If not present, the output file name is derived from the input file name by replacing the extension with .dae." );
    arguments.getApplicationUsage()->addCommandLineOption( "--display", "Opens a window and displays a physics simulation of the rigid body. Use for debugging only." );
    arguments.getApplicationUsage()->addCommandLineOption( "-h or --help", "Displays help text and command line documentation." );
    arguments.getApplicationUsage()->addCommandLineOption( "-v or --version", "Display the osgbBullet version string." );


    if( arguments.read( "-h" ) || arguments.read( "--help" ) )
    {
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    if (arguments.errors())
    {
        arguments.writeErrorMessages( osg::notify( osg::FATAL ) );
        return 1;
    }

    if ( arguments.argc() <= 1 )
    {
        arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    if( arguments.read( "-v" ) || arguments.read( "--version" ) )
    {
        osg::notify( osg::ALWAYS ) << osgbBullet::getVersionString() << std::endl << std::endl;
    }

    // Get all arguments.
    BroadphaseNativeTypes shapeType( BOX_SHAPE_PROXYTYPE );
    if( arguments.read( "--box" ) ) shapeType = BOX_SHAPE_PROXYTYPE;
    if( arguments.read( "--sphere" ) ) shapeType = SPHERE_SHAPE_PROXYTYPE;
    if( arguments.read( "--cylinder" ) ) shapeType = CYLINDER_SHAPE_PROXYTYPE;
    if( arguments.read( "--triMesh" ) ) shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
    if( arguments.read( "--convexTM" ) ) shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
    if( arguments.read( "--convexHull" ) ) shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;

    switch( shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: Box" << std::endl;
        break;
    case SPHERE_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: Sphere" << std::endl;
        break;
    case CYLINDER_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: Cylinder" << std::endl;
        break;
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: TriMesh" << std::endl;
        break;
    case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: ConvexTriMesh" << std::endl;
        break;
    case CONVEX_HULL_SHAPE_PROXYTYPE:
        osg::notify( osg::INFO ) << "osgbpp: ConvexHull" << std::endl;
        break;
    default:
        osg::notify( osg::FATAL ) << "osgbpp: Error, unknown shape type, using tri mesh." << std::endl;
        shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
        break;
    }

    std::string str;
    osgbBullet::AXIS axis( osgbBullet::Z );
    if ( arguments.read( "--axis", str ) )
    {
        if( (str.find( "X" ) != str.npos) || (str.find( "x" ) != str.npos) )
            axis = osgbBullet::X;
        else if( (str.find( "Y" ) != str.npos) || (str.find( "y" ) != str.npos) )
            axis = osgbBullet::Y;
        else if( (str.find( "Z" ) != str.npos) || (str.find( "z" ) != str.npos) )
            axis = osgbBullet::Z;
        else
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    switch( axis )
    {
    case osgbBullet::X:
        osg::notify( osg::INFO ) << "osgbpp: Axis: X" << std::endl;
        break;
    case osgbBullet::Y:
        osg::notify( osg::INFO ) << "osgbpp: Axis: Y" << std::endl;
        break;
    case osgbBullet::Z:
        osg::notify( osg::INFO ) << "osgbpp: Axis: Z" << std::endl;
        break;
    }


    float decimatorPercent( 1. );
    float decimatorMaxError( FLT_MAX );
    if ( arguments.read( "--decPercent", str ) )
    {
        if( sscanf( str.c_str(), "%f", &decimatorPercent ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
        if ( arguments.read( "--decMaxError", str ) )
        {
            if( sscanf( str.c_str(), "%f", &decimatorMaxError ) != 1 )
            {
                arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
                return 1;
            }
        }
    }
    if (decimatorPercent != 1.f )
        osg::notify( osg::INFO ) << "osgbpp: DecimatorOp: " << decimatorPercent << ", " << decimatorMaxError << std::endl;

    float simplifyPercent = 1.f;
    if ( arguments.read( "--simplify", str ) )
    {
        if( sscanf( str.c_str(), "%f", &simplifyPercent ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    if (simplifyPercent != 1.f )
        osg::notify( osg::INFO ) << "osgbpp: Simplify: " << simplifyPercent << std::endl;

    unsigned int vertexAggMaxVerts( 0 );
    osg::Vec3 vertexAggMinCellSize( 0., 0., 0. );
    if ( arguments.read( "--aggMaxVerts", str ) )
    {
        if( sscanf( str.c_str(), "%u", &vertexAggMaxVerts ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
        if ( arguments.read( "--aggMinCellSize", str ) )
        {
            char comma;
            std::istringstream oStr( str );
            oStr >> vertexAggMinCellSize[ 0 ] >> comma >>
                vertexAggMinCellSize[ 1 ] >> comma >>
                vertexAggMinCellSize[ 2 ];
        }
    }
    if (vertexAggMaxVerts > 0 )
        osg::notify( osg::INFO ) << "osgbpp: VertexAggOp: " << vertexAggMaxVerts << ", " << vertexAggMinCellSize << std::endl;

    const bool overall( arguments.read( "--overall" ) );
    if (overall)
        osg::notify( osg::INFO ) << "osgbpp: Overall" << std::endl;

    std::string nodeName;
    if ( arguments.read( "--name", nodeName ) )
        osg::notify( osg::INFO ) << "osgbpp: Node name: " << nodeName << std::endl;

    float mass( 1.f );
    if ( arguments.read( "--mass", str ) )
    {
        if( sscanf( str.c_str(), "%f", &mass ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    if (mass != 1.f )
        osg::notify( osg::INFO ) << "osgbpp: Mass: " << mass << std::endl;

    std::string outputFileName;
    if (! arguments.read( "-o", outputFileName ) )
    {
        // Find the first non-option and try to treat it like a file name.
        for( int pos = 1; pos < arguments.argc(); ++pos )
        {
            if ( ! arguments.isOption( pos ) )
            {
                outputFileName = osgDB::getNameLessExtension( arguments[ pos ] ) + ".dae";
                break;
            }
        }
    }
    osg::notify( osg::INFO ) << "osgbpp: Using output file name: " << outputFileName << std::endl;

    bool comSpecified;
    std::string comStr;
    osg::Vec3 com;
    if( comSpecified = arguments.read( "--com", comStr ) )
    {
        char comma;
        std::istringstream oStr( comStr );
        oStr >> com[ 0 ] >> comma >> com[ 1 ] >> comma >> com[ 2 ];
        osg::notify( osg::INFO ) << "osgbpp: Using center of mass: " << com << std::endl;
    }

    const bool display( arguments.read( "--display" ) );
    if (display)
        osg::notify( osg::INFO ) << "osgbpp: Display" << std::endl;



    osg::ref_ptr< osg::Node > model = osgDB::readNodeFiles( arguments );
    if( !model )
    {
        osg::notify( osg::FATAL ) << "Can't load input file(s)." << std::endl;
        return 1;
    }
    osg::notify( osg::INFO ) << "osgbpp: Loaded model(s)." << std::endl;


    osgbBullet::OSGToCollada converter;
    if( comSpecified )
        converter.setCenterOfMass( com );
    converter.setSceneGraph( model.get() );
    converter.setShapeType( shapeType );
    converter.setMass( mass );

    converter.setOverall( overall );
    converter.setNodeName( nodeName );
    converter.setAxis( axis );

    converter.convert( outputFileName );
    osg::notify( osg::INFO ) << "osgbpp: Completed Collada conversion." << std::endl;

    // TBD we can deallocate 'model' here, but don't want to deallocate 'converter' yet...

    if (!display)
        return 0;


    osg::ref_ptr< osgwTools::AbsoluteModelTransform > loadedModel( new osgwTools::AbsoluteModelTransform );
    {
        osg::Node* load = osgDB::readNodeFiles( arguments );
        if( !load )
        {
            osg::notify( osg::FATAL ) << "Can't load input file(s)." << std::endl;
            return 1;
        }
        loadedModel->addChild( load );
        if( mass != 0)
            loadedModel->setDataVariance( osg::Object::DYNAMIC );
    }
    osg::notify( osg::INFO ) << "osgbpp: Reloaded model(s) for display." << std::endl;

    btRigidBody* rb = converter.getRigidBody();
    osgbBullet::MotionState* motion = new osgbBullet::MotionState;
    motion->setTransform( loadedModel.get() );
    osg::BoundingSphere bs = loadedModel->getBound();

    if( comSpecified )
        motion->setCenterOfMass( com );
    else
        motion->setCenterOfMass( bs.center() );
    rb->setMotionState( motion );


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 10, 30, 800, 600 );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator();
    viewer.setCameraManipulator( tb );

    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild( loadedModel.get() );


    btDynamicsWorld* dynamicsWorld = initPhysics();
    dynamicsWorld->addRigidBody( rb );

    // Compute a reasonable ground plane size based on the bounding sphere radius.
    float dim = loadedModel->getBound()._radius * 1.5;
    osg::Vec3 cen = loadedModel->getBound()._center;
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
