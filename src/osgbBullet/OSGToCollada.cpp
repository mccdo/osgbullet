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

#include "osgbBullet/OSGToCollada.h"
#include "osgbBullet/CreationRecord.h"

#include <btBulletDynamicsCommon.h>
#include <BulletColladaConverter/ColladaConverter.h>

#include "osgbBullet/MotionState.h"
#include "osgbBullet/Utils.h"

#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/TransformAttributeFunctor>
#include <osg/Version>
#include <osg/io_utils>

using namespace osgbBullet;



class ProcessSceneGraph : public osg::NodeVisitor
{
public:
    ProcessSceneGraph( const bool overall, const std::string& nodeName,
        const BroadphaseNativeTypes shapeType, const AXIS axis )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _overall( overall ),
        _nodeName( nodeName ),
        _shapeType( shapeType ),
        _axis( axis ),
        _shape( NULL ),
        _process( false )
    {
        if (!_overall)
            _shape = new btCompoundShape;
    }

    void apply( osg::Node& node )
    {
        osg::notify( osg::DEBUG_INFO ) << "Node" << std::endl;

        // If this is the _nodeName-specified subgraph root, toggle _process to off after calling traverse()
        bool clearProcess( false );
        // Process the entire subgraph rooted at this node?
        bool found( _overall );

        if( !_nodeName.empty() && !_process )
        {
            // _nodeName was specified and we did _not_ already encounter this node.
            if ( _nodeName != node.getName() )
                // This node's name doesn't match _nodeName
                found = false;
            else
                // Match! 'found' is already set based on _overall,
                //  so set this to true in case _overall is false.
                _process = clearProcess = true;
        }

        if (found)
        {
            _shape = createShape( node );
            return;
        }

        traverse( node );

        if( clearProcess )
            _process = false;;
    }

    void apply( osg::Geode& node )
    {
        osg::notify( osg::DEBUG_INFO ) << "Geode" << std::endl;

        // We're at a Geode. Regardless of _overall, we will process it.
        bool found( true );
        if( !_nodeName.empty() && !_process )
        {
            // But if a nodeName was specified and we didn't turn on _process at a higher level,
            // then we process this node only if the name matches.
            found = ( _nodeName == node.getName() );
        }

        if (found)
            createAndAddShape( node );
    }

    btCollisionShape* getShape()
    {
        return( _shape );
    }

protected:
    void createAndAddShape( osg::Node& node )
    {
        osg::notify( osg::DEBUG_INFO ) << "In createAndAddShape" << std::endl;

        btCollisionShape* child = createShape( node );
        if (child)
        {
            btCompoundShape* master = dynamic_cast< btCompoundShape* >( _shape );
            btTransform transform; transform.setIdentity();
            master->addChildShape( transform, child );
        }
    }
    btCollisionShape* createShape( osg::Node& node )
    {
        osg::notify( osg::DEBUG_INFO ) << "In createShape" << std::endl;

        btCollisionShape* collision( NULL );
        osg::Vec3 center;

        switch( _shapeType )
        {
        case BOX_SHAPE_PROXYTYPE:
        {
            osg::ComputeBoundsVisitor cbv;
            node.accept( cbv );
            osg::BoundingBox bb = cbv.getBoundingBox();
            center = bb.center();
            collision = osgbBullet::btBoxCollisionShapeFromOSG( &node, &bb );
            break;
        }
        case SPHERE_SHAPE_PROXYTYPE:
        {
            osg::BoundingSphere bs = node.getBound();
            center = bs.center();
            collision = osgbBullet::btSphereCollisionShapeFromOSG( &node );
            break;
        }
        case CYLINDER_SHAPE_PROXYTYPE:
        {
            osg::BoundingSphere bs = node.getBound();
            center = bs.center();
            collision = osgbBullet::btCylinderCollisionShapeFromOSG( &node, _axis );
            break;
        }
        case TRIANGLE_MESH_SHAPE_PROXYTYPE:
        {
            // Do _not_ compute center of bounding sphere for tri meshes.
            collision = osgbBullet::btTriMeshCollisionShapeFromOSG( &node );
            break;
        }
        case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
        {
            // Do _not_ compute center of bounding sphere for tri meshes.
            collision = osgbBullet::btConvexTriMeshCollisionShapeFromOSG( &node );
            break;
        }
        case CONVEX_HULL_SHAPE_PROXYTYPE:
        {
            // Do _not_ compute center of bounding sphere for tri meshes.
            collision = osgbBullet::btConvexHullCollisionShapeFromOSG( &node );
            break;
        }
        default:
        {
            osg::notify( osg::FATAL ) << "OSGToCollada: Error, unknown shape type, using tri mesh." << std::endl;
            break;
        }
        }

        if( collision && (center != osg::Vec3( 0., 0., 0. )) )
        {
            btTransform trans; trans.setIdentity();
            trans.setOrigin( osgbBullet::asBtVector3( center ) );
            btCompoundShape* masterShape = new btCompoundShape();
            masterShape->addChildShape( trans, collision );
            collision = masterShape;
        }

        return( collision );
    }

    const bool _overall;
    const std::string& _nodeName;
    const BroadphaseNativeTypes _shapeType;
    const AXIS _axis;

    bool _process;

    btCollisionShape* _shape;
};

class FlattenTransforms : public osg::NodeVisitor
{
public:
    FlattenTransforms()
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
    {}

    void apply( osg::Transform& node )
    {
        // This is a Transform that isn't a MatrixTransform and isn't a PAT.
        // If it is not an AMT, display a warning.
        if( node.className() != std::string( "AbsoluteModelTransform" ) )
            osg::notify( osg::WARN ) << "OSGToCollada: Warning: Non-MatrixTransform encountered: (" <<
                node.className() << ") " << node.getName() << std::endl;
        traverse( node );
    }
    void apply( osg::MatrixTransform& node )
    {
        traverse( node );
        node.setMatrix( osg::Matrix::identity() );
    }
    void apply( osg::PositionAttitudeTransform& node )
    {
        traverse( node );
        node.setPosition(osg::Vec3(0.0f,0.0f,0.0f));
        node.setAttitude(osg::Quat());
        node.setPivotPoint(osg::Vec3(0.0f,0.0f,0.0f));
        node.setScale(osg::Vec3(1.0f,1.0f,1.0f));
    }

    void apply( osg::Geode& node )
    {
        osg::Matrix l2w = osg::computeLocalToWorld( getNodePath() );
        unsigned int idx;
        for( idx=0; idx<node.getNumDrawables(); idx++ )
        {
            osg::Drawable* draw( node.getDrawable( idx ) );

            osg::Geometry* geom( dynamic_cast< osg::Geometry* >( draw ) );
            if( geom )
            {
                // Requires 2.6.1, the necessary Geometry support didn't exist in 2.6.
                if( geom->containsSharedArrays() )
                    geom->duplicateSharedArrays();
            }

            flattenDrawable( draw, l2w );
        }
    }

protected:
    void flattenDrawable( osg::Drawable* drawable, const osg::Matrix& matrix )
    {
        if( drawable )
        {
            osgUtil::TransformAttributeFunctor tf(matrix);
            drawable->accept(tf);
            drawable->dirtyBound();
            drawable->dirtyDisplayList();
        }
    }
};


OSGToCollada::OSGToCollada()
{
    init();
}
OSGToCollada::OSGToCollada( CreationRecord& cr )
{
    _cr = new CreationRecord( cr );
}
OSGToCollada::~OSGToCollada()
{
}
void
OSGToCollada::init()
{
    _rigidBody = NULL;
    _cr = NULL;

    setSceneGraph( NULL );
    setCenterOfMass( osg::Vec3( 0., 0., 0. ) );
    setAutoComputeCenterOfMass( true );
    setScale( osg::Vec3( 1., 1., 1. ) );
    setShapeType( BOX_SHAPE_PROXYTYPE );
    setMass( 1.f );

    setOverall( true );
    setNodeName( std::string( "" ) );
    setAxis( osgbBullet::Z );
}


bool OSGToCollada::convert( const std::string& outputFileName )
{
    osg::BoundingSphere bs = getSceneGraph()->getBound();

    // Bullet collision shapes must be centered on the origin for correct
    // center of mass behavior. Calling code can call setCenterOfMass
    // to specify an arbitrary COM other than the origin.
    // Translate this subgraph so it is centered on the COM.
    osg::notify( osg::INFO ) << "OSGToCollada: ";
    if( getAutoComputeCenterOfMass() )
    {
        // Compute from bounding sphere.
        setCenterOfMass( bs.center() );
        // Kind of a HACK, set to true because above sets it to false.
        setAutoComputeCenterOfMass( true );
        osg::notify( osg::INFO ) << "Bounding sphere ";
    }
    else
    {
        // Use user-specified center of mass.
        osg::notify( osg::INFO ) << "User-defined ";
    }
    osg::notify( osg::INFO ) << "center of mass: " << getCenterOfMass() << std::endl;

    osg::Matrix m( osg::Matrix::translate( -getCenterOfMass() ) * osg::Matrix::scale( getScale() ) );
    osg::ref_ptr< osg::MatrixTransform > root = new osg::MatrixTransform( m );
    root->setDataVariance( osg::Object::STATIC );
    root->setName( "CenterOfMassOffset" );
    root->addChild( getSceneGraph() );


    // Flatten all transforms so that we don't need to multiply geometry by the current
    // OSG local to world matrix during traversal.
    // NOTE: Must remove loaded ProxyNodes in order to allow
    //   all transforms to be flattened.
    osg::notify( osg::INFO ) << "OSGToCollada: Flattening transforms." << std::endl;
    FlattenTransforms ft;
    root->accept( ft );


    osg::notify( osg::INFO ) << "OSGToCollada: ProcessSceneGraph." << std::endl;
    ProcessSceneGraph psg( getOverall(), getNodeName(), getShapeType(), getAxis() );
    root->accept( psg );

    _rigidBody = createRigidBody( getMass(), psg.getShape(),
        getCenterOfMass(), getSceneGraph() );


    if (!_rigidBody)
    {
        osg::notify( osg::FATAL ) << "OSGToCollada: Unable to create physics data." << std::endl;
        return false;
    }
    else if (outputFileName.empty())
    {
        osg::notify( osg::INFO ) << "OSGToCollada: No output file name, not writing DAE." << std::endl;
        return true;
    }
    else
    {
        btDynamicsWorld* dynamicsWorld = initPhysics();
        dynamicsWorld->addRigidBody( _rigidBody );

        ColladaConverter* cc = new ColladaConverter( dynamicsWorld );
        // TBD Not available in 2.75 Bullet release, maybe next one.
        //cc->setVerbosity( ColladaConverter::SILENT );
        cc->save( outputFileName.c_str() );

        // clean up.
        delete cc;

        dynamicsWorld->removeRigidBody( _rigidBody );

        delete dynamicsWorld->getBroadphase();
        delete dynamicsWorld->getConstraintSolver();
        delete dynamicsWorld->getDispatcher();
        delete dynamicsWorld;

        return true;
    }
}

btRigidBody*
OSGToCollada::getRigidBody()
{
    return( _rigidBody );
}

CreationRecord*
OSGToCollada::getOrCreateCreationRecord()
{
    if( !_cr.valid() )
        _cr = new CreationRecord;
    return( _cr.get() );
}
const CreationRecord*
OSGToCollada::getOrCreateCreationRecord() const
{
    if( !_cr.valid() )
        _cr = new CreationRecord;
    return( _cr.get() );
}

void
OSGToCollada::setSceneGraph( osg::Node* sg )
{
    getOrCreateCreationRecord()->_sceneGraph = sg;
}
osg::Node*
OSGToCollada::getSceneGraph() const
{
    return( getOrCreateCreationRecord()->_sceneGraph );
}

void
OSGToCollada::setCenterOfMass( const osg::Vec3& com )
{
    getOrCreateCreationRecord()->_com = com;
    setAutoComputeCenterOfMass( false );
}
const osg::Vec3&
OSGToCollada::getCenterOfMass() const
{
    return( getOrCreateCreationRecord()->_com );
}
void
OSGToCollada::setAutoComputeCenterOfMass( bool compute )
{
    getOrCreateCreationRecord()->_comSet = !compute;
}
bool
OSGToCollada::getAutoComputeCenterOfMass() const
{
    return( !getOrCreateCreationRecord()->_comSet );
}

void
OSGToCollada::setShapeType( const BroadphaseNativeTypes shapeType )
{
    getOrCreateCreationRecord()->_shapeType = shapeType;
}
BroadphaseNativeTypes
OSGToCollada::getShapeType() const
{
    return( getOrCreateCreationRecord()->_shapeType );
}

void
OSGToCollada::setMass( float mass )
{
    getOrCreateCreationRecord()->_mass = mass;
}
float
OSGToCollada::getMass() const
{
    return( getOrCreateCreationRecord()->_mass );
}


void
OSGToCollada::setOverall( bool overall )
{
    getOrCreateCreationRecord()->_overall = overall;
}
bool
OSGToCollada::getOverall() const
{
    return( getOrCreateCreationRecord()->_overall );
}

void
OSGToCollada::setNodeName( const std::string& nodeName )
{
    getOrCreateCreationRecord()->_nodeName = nodeName;
}
const std::string&
OSGToCollada::getNodeName() const
{
    return( getOrCreateCreationRecord()->_nodeName );
}

void
OSGToCollada::setAxis( osgbBullet::AXIS axis )
{
    getOrCreateCreationRecord()->_axis = axis;
}
osgbBullet::AXIS
OSGToCollada::getAxis() const
{
    return( getOrCreateCreationRecord()->_axis );
}

void
OSGToCollada::setScale( const osg::Vec3& scale )
{
    getOrCreateCreationRecord()->_scale = scale;
}
osg::Vec3
OSGToCollada::getScale() const
{
    return( getOrCreateCreationRecord()->_scale );
}


btDynamicsWorld*
OSGToCollada::initPhysics()
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

btRigidBody*
OSGToCollada::createRigidBody( btScalar mass,
                              btCollisionShape* shape, const osg::Vec3& centerOfMass, osg::Node* node )
{
    if( (shape == NULL) || (node == NULL) )
        return NULL;

    btTransform startTransform; startTransform.setIdentity();

	btVector3 localInertia( 0, 0, 0 );
	const bool isDynamic = (mass != 0.f);
	if (isDynamic)
		shape->calculateLocalInertia( mass, localInertia );

    osgbBullet::MotionState* motion( NULL );
    osg::Transform* trans = dynamic_cast< osg::Transform* >( node );
    if( trans != NULL )
    {
        motion = new osgbBullet::MotionState();
        motion->setTransform( trans );
        motion->setCenterOfMass( centerOfMass );
        // TBD. motion->setScale?
    }

    btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motion, shape, localInertia );
    rbInfo.m_friction = btScalar( 1. );
	return( new btRigidBody( rbInfo ) );
}

