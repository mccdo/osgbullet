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

#include <osgbDynamics/Constraints.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/Utils.h>

#include <osg/Object>
#include <osg/Notify>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>


namespace osgbDynamics
{


Constraint::Constraint()
  : osg::Object(),
    _constraint( NULL )
{
}
Constraint::Constraint( const Constraint& rhs, const osg::CopyOp& copyop )
  : osg::Object( rhs, copyop ),
    _constraint( NULL )
{
}
Constraint::~Constraint()
{
    // Deleting the constraint is up to the calling code. Something like this:
    //delete osgbDynamics::Constraint::getConstraint();
}



SliderConstraint::SliderConstraint()
  : Constraint(),
    _rbA( NULL ),
    _rbB( NULL )
{
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint(),
    _rbA( rbA ),
    _rbB( rbB )
{
    createConstraint();
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit )
  : Constraint(),
    _rbA( rbA ),
    _rbB( rbB ),
    _rbAXform( rbAXform ),
    _rbBXform( rbBXform ),
    _slideAxisInA( slideAxisInA ),
    _slideLimit( slideLimit )
{
    createConstraint();
}
SliderConstraint::SliderConstraint( const SliderConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _rbA( rhs._rbA ),
    _rbB( rhs._rbB ),
    _rbAXform( rhs._rbAXform ),
    _rbBXform( rhs._rbBXform ),
    _slideAxisInA( rhs._slideAxisInA ),
    _slideLimit( rhs._slideLimit )
{
}
SliderConstraint::~SliderConstraint()
{
    // Deleting the constraint is up to the calling code.
}

btSliderConstraint* SliderConstraint::getAsBtSlider() const
{
    return( static_cast< btSliderConstraint* >( _constraint ) );
}

void SliderConstraint::setRigidBodies( btRigidBody* rbA, btRigidBody* rbB )
{
    _rbA = rbA;
    _rbB = rbB;
    createConstraint();
}
void SliderConstraint::setAXform( const osg::Matrix& rbAXform )
{
    _rbAXform = rbAXform;
    createConstraint();
}
void SliderConstraint::setBXform( const osg::Matrix& rbBXform )
{
    _rbBXform = rbBXform;
    createConstraint();
}
void SliderConstraint::setAxisInA( const osg::Vec3& axisInA )
{
    _slideAxisInA = axisInA;
    createConstraint();
}
void SliderConstraint::setLimit( const osg::Vec2& limit )
{
    _slideLimit = limit;
    createConstraint();
}

void SliderConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint )
        delete _constraint;

    const osg::Vec3 bulletSliderAxis( 1., 0., 0. );


    // Compute a matrix that transforms B's collision shape origin and x axis
    // to A's origin and slide axis.
    //   1. Matrix to align the (slider constraint) x axis with A's slide axis.
    const osg::Matrix axisRotate( osg::Matrix::rotate( bulletSliderAxis, _slideAxisInA ) );
    //
    //   2. Inverse B center of mass offset.
    osg::Vec3 bCom;
    osgbDynamics::MotionState* motion;
    if( _rbB != NULL )
    {
        motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
            return;
        }
        bCom = motion->getCenterOfMass();
    }
    const osg::Matrix invBCOM( osg::Matrix::translate( -( bCom ) ) );
    //
    //   3. Transform from B's origin to A's origin.
    const osg::Matrix rbBToRbA( osg::Matrix::inverse( _rbBXform ) * _rbAXform );
    //
    //   4. The final rbB frame matrix.
    btTransform rbBFrame = osgbCollision::asBtTransform(
        axisRotate * invBCOM * rbBToRbA );


    // Compute a matrix that transforms A's collision shape origin and x axis
    // to A's origin and drawerAxis.
    //   1. A's center of mass offset.
    motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
        return;
    }
    const osg::Matrix invACOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
    //
    //   2. The final rbA frame matrix.
    btTransform rbAFrame = osgbCollision::asBtTransform(
        axisRotate * invACOM );


    btSliderConstraint* sc;
    if( _rbB != NULL )
        sc = new btSliderConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        sc = new btSliderConstraint( *_rbA, rbAFrame, true );
    sc->setLowerLinLimit( _slideLimit[0] );
    sc->setUpperLinLimit( _slideLimit[1] );
    _constraint = sc;
}


// osgbDynamics
}
