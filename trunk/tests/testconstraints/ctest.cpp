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

#include "ctest.h"
#include <osgbDynamics/Constraints.h>
#include <osgbDynamics/RigidBody.h>
#include <osgwTools/AbsoluteModelTransform.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Notify>

#include <btBulletDynamicsCommon.h>


// Shorthand to display a message and return an error.
#define ERROR( name, msg ) \
    { \
        osg::notify( osg::WARN ) << name << " " << msg << std::endl; \
        return( 1 ); \
    }


int runCTest()
{
    const std::string fileName( "testconstraint.osg" );

    // Create two rigid bodies for testing.

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osg::Node* node = osgDB::readNodeFile( "tetra.osg" );
    if( node == NULL )
        ERROR("Init:","Can't load model data file.");
    osg::Matrix aXform = osg::Matrix::translate( 4., 2., 0. );

    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    amt->addChild( node );
    root->addChild( amt );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
        cr->_mass = .5;
    cr->_parentTransform = aXform;
    btRigidBody* rbA = osgbDynamics::createRigidBody( cr.get() );


    node = osgDB::readNodeFile( "block.osg" );
    if( node == NULL )
        ERROR("Init:","Can't load model data file.");
    osg::Matrix bXform = osg::Matrix::identity();

    amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    amt->addChild( node );
    root->addChild( amt );

    cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
        cr->_mass = 4.;
    cr->_parentTransform = bXform;
    btRigidBody* rbB = osgbDynamics::createRigidBody( cr.get() );

    //
    // SliderConstraint
    {
        const std::string name( "TwistSliderConstraint" );
        osg::Vec3 axis( 0., 0., 1. );
        osg::Vec2 limits( -4., 4. );
        osg::ref_ptr< osgbDynamics::SliderConstraint > cons = new osgbDynamics::SliderConstraint(
            rbA, aXform, rbB, bXform, axis, limits );

        if( cons->getAsBtSlider() == NULL )
            ERROR(name,"won't typecast as btSliderConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(name,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(name,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::SliderConstraint > cons2 = dynamic_cast<
            osgbDynamics::SliderConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(name,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            ERROR(name,"failed to match.");
    }

    //
    // TwistSliderConstraint
    {
        const std::string name( "TwistSliderConstraint" );
        osg::Vec3 axis( 0., 0., 1. );
        osg::Vec2 limits( -4., 4. );
        osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons = new osgbDynamics::TwistSliderConstraint(
            rbA, aXform, rbB, bXform, axis, limits );

        if( cons->getAsBtSlider() == NULL )
            ERROR(name,"won't typecast as btSliderConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(name,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(name,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::SliderConstraint > cons2 = dynamic_cast<
            osgbDynamics::SliderConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(name,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            ERROR(name,"failed to match.");
    }

    //
    // BallAndSocketConstraint
    {
        const std::string name( "BallAndSocketConstraint" );
        osg::Vec3 point( -5., 5., 3. );
        osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons = new osgbDynamics::BallAndSocketConstraint(
            rbA, aXform, rbB, bXform, point );

        if( cons->getAsBtPoint2Point() == NULL )
            ERROR(name,"won't typecast as btPoint2PointConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(name,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(name,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons2 = dynamic_cast<
            osgbDynamics::BallAndSocketConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(name,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            ERROR(name,"failed to match.");
    }

    //
    // FixedConstraint
    {
        const std::string name( "FixedConstraint" );
        osg::ref_ptr< osgbDynamics::FixedConstraint > cons = new osgbDynamics::FixedConstraint(
            rbA, aXform, rbB, bXform );

        if( cons->getAsBtGeneric6Dof() == NULL )
            ERROR(name,"won't typecast as btGeneric6DofConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(name,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(name,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::FixedConstraint > cons2 = dynamic_cast<
            osgbDynamics::FixedConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(name,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            ERROR(name,"failed to match.");
    }

    return( 0 );
}
