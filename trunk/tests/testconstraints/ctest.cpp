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
#define ERROR( msg ) \
    { \
        osg::notify( osg::WARN ) << msg << std::endl; \
        return( 1 ); \
    }


int runCTest()
{
    const std::string fileName( "testconstraint.osg" );

    // Create two rigid bodies for testing.

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osg::Node* node = osgDB::readNodeFile( "tetra.osg" );
    if( node == NULL )
        ERROR("Can't load model data file.");
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
        ERROR("Can't load model data file.");
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
        osg::Vec3 axis( 0., 0., 1. );
        osg::Vec2 limits( -4., 4. );
        osg::ref_ptr< osgbDynamics::SliderConstraint > sc = new osgbDynamics::SliderConstraint(
            rbA, aXform, rbB, bXform, axis, limits );

        if( sc->getAsBtSlider() != NULL )
            ERROR("SliderConstraint won't typecast as btSliderConstraint.");

        if( !( osgDB::writeObjectFile( *sc, fileName ) ) )
            ERROR("SliderConstraint writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR("SliderConstraint readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::SliderConstraint > sc2 = dynamic_cast<
            osgbDynamics::SliderConstraint* >( obj );
        if( !( sc2.valid() ) )
            ERROR("SliderConstraint dynamic_cast after readObjectFile failed.");

        if( *sc2 != *sc )
            ERROR("SliderConstraints failed to match.");
    }

    return( 0 );
}
