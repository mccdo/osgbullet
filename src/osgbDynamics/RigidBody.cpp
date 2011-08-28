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

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>

#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/BoundingSphere>
#include <osg/Notify>
#include <osg/ref_ptr>
#include <osg/io_utils>


namespace osgbDynamics
{


btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr )
{
    osg::Node* root = cr->_sceneGraph;
    if( root == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: CreationRecord has NULL scene graph." << std::endl;
        return( NULL );
    }

    osg::BoundingSphere bs = root->getBound();


    // Bullet collision shapes must be centered on the origin for correct
    // center of mass behavior. Calling code should call
    // CreationRecord::setCenterOfMass() to specify COM. Otherwise, this
    // function uses the bounding volume center as the COM.
    // Translate this subgraph so it is centered on the COM.
    osg::notify( osg::DEBUG_FP ) << "createRigidBody: ";
    osg::Vec3 com;
    if( !( cr->_comSet ) )
    {
        // Compute from bounding sphere.
        com = bs.center();
        osg::notify( osg::DEBUG_FP ) << "Bounding sphere ";
    }
    else
    {
        // Use user-specified center of mass.
        com = cr->_com;
        osg::notify( osg::DEBUG_FP ) << "User-defined ";
    }
    osg::notify( osg::DEBUG_FP ) << "center of mass: " << com << std::endl;

    osg::Matrix m( osg::Matrix::translate( -com ) * osg::Matrix::scale( cr->_scale ) );
    osg::ref_ptr< osg::MatrixTransform > mtRoot = new osg::MatrixTransform( m );
    mtRoot->setDataVariance( osg::Object::STATIC );
    mtRoot->setName( "CenterOfMass+Scale" );
    mtRoot->addChild( root );


    osg::notify( osg::DEBUG_FP ) << "createRigidBody: Creating collision shape." << std::endl;
    btCompoundShape* shape = osgbCollision::btCompoundShapeFromOSGGeodes( mtRoot.get(),
        cr->_shapeType, cr->_axis );
    if( shape == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: btCompoundShapeFromOSGGeodes returned NULL." << std::endl;
        return( NULL );
    }

    return( createRigidBody( cr, shape ) );
}

btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr, btCollisionShape* shape )
{
    osg::Node* root = cr->_sceneGraph;
    if( root == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: CreationRecord has NULL scene graph." << std::endl;
        return( NULL );
    }


    osg::notify( osg::DEBUG_FP ) << "createRigidBody: Creating rigid body." << std::endl;
	btVector3 localInertia( 0, 0, 0 );
    const bool isDynamic = ( cr->_mass != 0.f );
	if( isDynamic )
		shape->calculateLocalInertia( cr->_mass, localInertia );

    btRigidBody::btRigidBodyConstructionInfo rbInfo( cr->_mass, NULL, shape, localInertia );
    rbInfo.m_friction = btScalar( 1. );
	btRigidBody* rb = new btRigidBody( rbInfo );
    if( rb == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: Created a NULL btRigidBody." << std::endl;
        return( NULL );
    }

    osg::Transform* trans = dynamic_cast< osg::Transform* >( root );
    if( trans != NULL )
    {
        osgbDynamics::MotionState* motion = new osgbDynamics::MotionState();
        motion->setTransform( trans );

        osg::Vec3 com;
        if( !( cr->_comSet ) )
            com = root->getBound().center();
        else
            com = cr->_com;
        motion->setCenterOfMass( com );
        rb->setMotionState( motion );
    }

    return( rb );
}


// osgbDynamics
}
