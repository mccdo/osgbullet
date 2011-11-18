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
    _rbA( NULL ),
    _rbB( NULL ),
    _constraint( NULL ),
    _dirty( true )
{
}
Constraint::Constraint( btRigidBody* rbA, btRigidBody* rbB )
  : osg::Object(),
    _rbA( rbA ),
    _rbB( rbB ),
    _constraint( NULL ),
    _dirty( true )
{
}
Constraint::Constraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform )
  : _rbA( rbA ),
    _rbB( rbB ),
    _rbAXform( rbAXform ),
    _rbBXform( rbBXform ),
    _constraint( NULL ),
    _dirty( true )
{
}
Constraint::Constraint( const Constraint& rhs, const osg::CopyOp& copyop )
  : osg::Object( rhs, copyop ),
    _rbA( rhs._rbA ),
    _rbB( rhs._rbB ),
    _rbAXform( rhs._rbAXform ),
    _rbBXform( rhs._rbBXform ),
    _constraint( rhs._constraint ),
    _dirty( rhs._dirty )
{
}
Constraint::~Constraint()
{
    // Deleting the constraint is up to the calling code. Something like this:
    //delete osgbDynamics::Constraint::getConstraint();
}

btTypedConstraint* Constraint::getConstraint() const
{
    if( getDirty() || ( _constraint == NULL ) )
    {
        Constraint* nonConst = const_cast< Constraint* >( this );
        nonConst->createConstraint();
    }

    return( _constraint );
}

void Constraint::setRigidBodies( btRigidBody* rbA, btRigidBody* rbB )
{
    _rbA = rbA;
    _rbB = rbB;
    setDirty();
}
void Constraint::setAXform( const osg::Matrix& rbAXform )
{
    _rbAXform = rbAXform;
    setDirty();
}
void Constraint::setBXform( const osg::Matrix& rbBXform )
{
    _rbBXform = rbBXform;
    setDirty();
}

bool Constraint::operator==( const Constraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool Constraint::operator!=( const Constraint& rhs ) const
{
    return(
        ( _rbAXform != rhs._rbAXform ) ||
        ( _rbBXform != rhs._rbBXform )
    );
}




SliderConstraint::SliderConstraint()
  : Constraint()
{
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
    setDirty();
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit )
  : Constraint( rbA, rbAXform ),
    _axis( slideAxisInA ),
    _slideLimit( slideLimit )
{
    setDirty();
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( slideAxisInA ),
    _slideLimit( slideLimit )
{
    setDirty();
}
SliderConstraint::SliderConstraint( const SliderConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis),
    _slideLimit( rhs._slideLimit )
{
}
SliderConstraint::~SliderConstraint()
{
    // Deleting the constraint is up to the calling code.
}

btSliderConstraint* SliderConstraint::getAsBtSlider() const
{
    return( static_cast< btSliderConstraint* >( getConstraint() ) );
}

void SliderConstraint::setAxis( const osg::Vec3& axis )
{
    _axis = axis;
    setDirty();
}
void SliderConstraint::setLimit( const osg::Vec2& limit )
{
    _slideLimit = limit;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btSliderConstraint* cons = getAsBtSlider();
        cons->setLowerLinLimit( _slideLimit[ 0 ] );
        cons->setUpperLinLimit( _slideLimit[ 1 ] );
    }
    else
        setDirty();
}

bool SliderConstraint::operator==( const SliderConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool SliderConstraint::operator!=( const SliderConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _slideLimit != rhs._slideLimit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
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


    // Transform the world coordinate axis into A's local coordinates.
    osg::Matrix aOrient = _rbAXform;
    aOrient.setTrans( 0., 0., 0. );
    const osg::Vec3 axisInA = _axis * osg::Matrix::inverse( aOrient );

    // Compute a matrix to align the slider constraint axis with A's slide axis.
    const osg::Vec3 bulletSliderAxis( 1., 0., 0. );
    const osg::Matrix axisRotate( osg::Matrix::rotate( bulletSliderAxis, axisInA ) );


    btTransform rbBFrame; // OK to not initialize.
    if( _rbB != NULL )
    {
        // Compute a matrix that transforms B's collision shape origin and x axis
        // to A's origin and slide axis.
        //
        //   1. Inverse B center of mass offset.
        osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
            return;
        }
        const osg::Vec3 bCom = motion->getCenterOfMass();
        const osg::Matrix invBCOM( osg::Matrix::translate( -( bCom ) ) );
        //
        //   2. Transform from B's origin to A's origin.
        const osg::Matrix rbBToRbA( osg::Matrix::inverse( _rbBXform ) * _rbAXform );
        //
        //   3. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform(
            axisRotate * invBCOM * rbBToRbA );
    }


    // Compute a matrix that transforms A's collision shape origin and x axis
    // to A's origin and drawerAxis.
    //   1. A's center of mass offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
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


    btSliderConstraint* cons;
    if( _rbB != NULL )
        cons = new btSliderConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btSliderConstraint( *_rbA, rbAFrame, true );
    const btScalar loLimit = _slideLimit[ 0 ];
    const btScalar hiLimit = _slideLimit[ 1 ];
    cons->setLowerLinLimit( loLimit );
    cons->setUpperLinLimit( hiLimit );
    _constraint = cons;

    setDirty( false );
}




TwistSliderConstraint::TwistSliderConstraint()
  : SliderConstraint()
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : SliderConstraint( rbA, rbB )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit )
  : SliderConstraint( rbA, rbAXform, rbB, rbBXform, slideAxisInA, slideLimit )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit )
  : SliderConstraint( rbA, rbAXform, slideAxisInA, slideLimit )
{
}
TwistSliderConstraint::TwistSliderConstraint( const TwistSliderConstraint& rhs, const osg::CopyOp& copyop )
  : SliderConstraint( rhs, copyop )
{
}
TwistSliderConstraint::~TwistSliderConstraint()
{
}

void TwistSliderConstraint::createConstraint()
{
    // Create the constraint using the base class.
    SliderConstraint::createConstraint();
    if( _constraint == NULL )
        return;

    // Base class produces a btSliderConstraint.
    btSliderConstraint* cons = getAsBtSlider();
    // All we need to do is disable the angular constraint that exists in
    // the btSliderConstraint by default.
    cons->setLowerAngLimit( -osg::PI );
    cons->setUpperAngLimit( osg::PI );

    setDirty( false );
}




LinearSpringConstraint::LinearSpringConstraint()
  : Constraint()
{
}
LinearSpringConstraint::LinearSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
LinearSpringConstraint::LinearSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform )
  : Constraint( rbA, rbAXform )
{
}
LinearSpringConstraint::LinearSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform )
  : Constraint( rbA, rbAXform, rbB, rbBXform )
{
}
LinearSpringConstraint::LinearSpringConstraint( const LinearSpringConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop )
{
}
LinearSpringConstraint::~LinearSpringConstraint()
{
}

btGeneric6DofSpringConstraint* LinearSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void LinearSpringConstraint::createConstraint()
{
}




AngleSpringConstraint::AngleSpringConstraint()
  : Constraint()
{
}
AngleSpringConstraint::AngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
AngleSpringConstraint::AngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform )
  : Constraint( rbA, rbAXform )
{
}
AngleSpringConstraint::AngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform )
  : Constraint( rbA, rbAXform, rbB, rbBXform )
{
}
AngleSpringConstraint::AngleSpringConstraint( const AngleSpringConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop )
{
}
AngleSpringConstraint::~AngleSpringConstraint()
{
}

btGeneric6DofSpringConstraint* AngleSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void AngleSpringConstraint::createConstraint()
{
}




LinearAngleSpringConstraint::LinearAngleSpringConstraint()
  : Constraint()
{
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform )
  : Constraint( rbA, rbAXform )
{
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform )
  : Constraint( rbA, rbAXform, rbB, rbBXform )
{
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( const LinearAngleSpringConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop )
{
}
LinearAngleSpringConstraint::~LinearAngleSpringConstraint()
{
}

btGeneric6DofSpringConstraint* LinearAngleSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void LinearAngleSpringConstraint::createConstraint()
{
}




FixedConstraint::FixedConstraint()
  : Constraint()
{
}
FixedConstraint::FixedConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
    setDirty();
}
FixedConstraint::FixedConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform )
  : Constraint( rbA, rbAXform, rbB, rbBXform )
{
    setDirty();
}
FixedConstraint::FixedConstraint( const FixedConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop )
{
}
FixedConstraint::~FixedConstraint()
{
}

btGeneric6DofConstraint* FixedConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void FixedConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint )
        delete _constraint;


    btTransform rbBFrame; // OK to not initialize.
    if( _rbB != NULL )
    {
        // Create a matrix that puts A in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
            return;
        }
        const osg::Vec3 bCom = motion->getCenterOfMass();
        const osg::Matrix invBCOM( osg::Matrix::translate( -( bCom ) ) );
        //
        //   3. Transform from B's origin to A's origin.
        const osg::Matrix rbBToRbA( osg::Matrix::inverse( _rbBXform ) * _rbAXform );
        //
        //   4. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform(
            invBCOM * rbBToRbA );
    }


    // A's reference frame is just the COM offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
        return;
    }
    const osg::Matrix invACOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
    btTransform rbAFrame = osgbCollision::asBtTransform( invACOM );


    btGeneric6DofConstraint* cons;
    if( _rbB != NULL )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );
    _constraint = cons;

    setDirty( false );
}




PlanarConstraint::PlanarConstraint()
  : Constraint()
{
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const osg::Vec2& loLimit, const osg::Vec2& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbB ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec2& loLimit, const osg::Vec2& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec2& loLimit, const osg::Vec2& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( const PlanarConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _loLimit( rhs._loLimit ),
    _hiLimit( rhs._hiLimit ),
    _orient( rhs._orient )
{
    setDirty( true );
}
PlanarConstraint::~PlanarConstraint()
{
}

btGeneric6DofConstraint* PlanarConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void PlanarConstraint::setLowLimit( const osg::Vec2& loLimit )
{
    _loLimit = loLimit;
    setDirty( true );
}
void PlanarConstraint::setHighLimit( const osg::Vec2& hiLimit )
{
    _hiLimit = hiLimit;
    setDirty( true );
}
void PlanarConstraint::setOrientation( const osg::Matrix& orient )
{
    _orient = orient;
    setDirty( true );
}

bool PlanarConstraint::operator==( const PlanarConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool PlanarConstraint::operator!=( const PlanarConstraint& rhs ) const
{
    return(
        ( _loLimit != rhs._loLimit ) ||
        ( _hiLimit != rhs._hiLimit ) ||
        ( _orient != rhs._orient ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void PlanarConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint )
        delete _constraint;


    btTransform rbBFrame; // OK to not initialize.
    if( _rbB != NULL )
    {
        // Create a matrix that puts A in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
            return;
        }
        const osg::Vec3 bCom = motion->getCenterOfMass();
        const osg::Matrix invBCOM( osg::Matrix::translate( -( bCom ) ) );
        //
        //   3. Transform from B's origin to A's origin.
        const osg::Matrix rbBToRbA( osg::Matrix::inverse( _rbBXform ) * _rbAXform );
        //
        //   4. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform(
            invBCOM * rbBToRbA );
    }


    // A's reference frame is just the COM offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
        return;
    }
    const osg::Matrix invACOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
    btTransform rbAFrame = osgbCollision::asBtTransform( 
        osg::Matrix::inverse( _rbAXform ) * invACOM );


    btGeneric6DofConstraint* cons;
    if( _rbB != NULL )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, true );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, false );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );

    const osg::Vec3 loLimit( _loLimit[ 0 ], _loLimit[ 1 ], 0. );
    const osg::Vec3 hiLimit( _hiLimit[ 0 ], _hiLimit[ 1 ], 0. );
    cons->setLinearLowerLimit( osgbCollision::asBtVector3( loLimit ) );
    cons->setLinearUpperLimit( osgbCollision::asBtVector3( hiLimit ) );

    _constraint = cons;

    setDirty( false );
}




BoxConstraint::BoxConstraint()
  : Constraint()
{
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const osg::Vec3& loLimit, const osg::Vec3& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbB ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& loLimit, const osg::Vec3& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& loLimit, const osg::Vec3& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( const BoxConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _loLimit( rhs._loLimit ),
    _hiLimit( rhs._hiLimit ),
    _orient( rhs._orient )
{
    setDirty( true );
}
BoxConstraint::~BoxConstraint()
{
}

btGeneric6DofConstraint* BoxConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void BoxConstraint::setLowLimit( const osg::Vec3& loLimit )
{
    _loLimit = loLimit;
    setDirty( true );
}
void BoxConstraint::setHighLimit( const osg::Vec3& hiLimit )
{
    _hiLimit = hiLimit;
    setDirty( true );
}
void BoxConstraint::setOrientation( const osg::Matrix& orient )
{
    _orient = orient;
    setDirty( true );
}

bool BoxConstraint::operator==( const BoxConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool BoxConstraint::operator!=( const BoxConstraint& rhs ) const
{
    return(
        ( _loLimit != rhs._loLimit ) ||
        ( _hiLimit != rhs._hiLimit ) ||
        ( _orient != rhs._orient ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void BoxConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint )
        delete _constraint;


    btTransform rbBFrame; // OK to not initialize.
    if( _rbB != NULL )
    {
        // Create a matrix that puts A in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
            return;
        }
        const osg::Vec3 bCom = motion->getCenterOfMass();
        const osg::Matrix invBCOM( osg::Matrix::translate( -( bCom ) ) );
        //
        //   3. Transform from B's origin to A's origin.
        const osg::Matrix rbBToRbA( osg::Matrix::inverse( _rbBXform ) * _rbAXform );
        //
        //   4. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform(
            invBCOM * rbBToRbA );
    }


    // A's reference frame is just the COM offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
        return;
    }
    const osg::Matrix invACOM( osg::Matrix::translate( -( motion->getCenterOfMass() ) ) );
    btTransform rbAFrame = osgbCollision::asBtTransform( invACOM );


    btGeneric6DofConstraint* cons;
    if( _rbB != NULL )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );

    cons->setLinearLowerLimit( osgbCollision::asBtVector3( _loLimit ) );
    cons->setLinearUpperLimit( osgbCollision::asBtVector3( _hiLimit ) );

    _constraint = cons;

    setDirty( false );
}




HingeConstraint::HingeConstraint()
  : Constraint(),
    _axis( osg::Vec3( 0., 0., 1. ) ),
    _pivotPoint( osg::Vec3( 0., 0., 0. ) ),
    _limit( osg::Vec2( -osg::PI, osg::PI ) )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const osg::Vec3& axis, const osg::Vec3& pivotPoint,
        const osg::Vec2& limit )
  : Constraint( rbA, rbB ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& axis, const osg::Vec3& pivotPoint,
        const osg::Vec2& limit )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& axis, const osg::Vec3& pivotPoint,
        const osg::Vec2& limit )
  : Constraint( rbA, rbAXform ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( const HingeConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _limit( rhs._limit )
{
}
HingeConstraint::~HingeConstraint()
{
}

btHingeConstraint* HingeConstraint::getAsBtHinge() const
{
    return( static_cast< btHingeConstraint* >( getConstraint() ) );
}

void HingeConstraint::setAxis( const osg::Vec3& axis )
{
    setDirty( true );
}
void HingeConstraint::setPivotPoint( const osg::Vec3& limit )
{
    setDirty( true );
}
void HingeConstraint::setLimit( const osg::Vec2& limit )
{
    _limit = limit;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btHingeConstraint* cons = getAsBtHinge();
        cons->setLimit( _limit[ 0 ], _limit[ 1 ] );
    }
    else
        setDirty();
}

bool HingeConstraint::operator==( const HingeConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool HingeConstraint::operator!=( const HingeConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _pivotPoint != rhs._pivotPoint ) ||
        ( _limit != rhs._limit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}


void HingeConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint )
        delete _constraint;


    // Pivot point and pivot axis are both in the gate's object space.
    // Note that the gate is COM-adjusted, so the pivot point must also be
    // in the gate's COM-adjusted object space.
    // TBD extract this from hinge data fine.

    btVector3 pivotInA( osgbCollision::asBtVector3( _pivotPoint ) );
    btVector3 axisInA( osgbCollision::asBtVector3( _axis ) );
    btVector3 pivotInB( osgbCollision::asBtVector3( _pivotPoint ) );
    btVector3 axisInB( osgbCollision::asBtVector3( _axis ) );

    btHingeConstraint* hinge;
    if( _rbB != NULL )
        hinge = new btHingeConstraint( *_rbA, *_rbB,
                pivotInA, pivotInB, axisInA, axisInB, false );
    else
        hinge = new btHingeConstraint( *_rbA, pivotInA, axisInA, false );

    hinge->setLimit( _limit[ 0 ], _limit[ 1 ] );

    _constraint = hinge;
}




CardanConstraint::CardanConstraint()
  : Constraint()
{
}
CardanConstraint::CardanConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
CardanConstraint::CardanConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& axisA, const osg::Vec3& axisB )
  : Constraint( rbA, rbAXform ),
    _axisA( axisA ),
    _axisB( axisB )
{
}
CardanConstraint::CardanConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& axisA, const osg::Vec3& axisB )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axisA( axisA ),
    _axisB( axisB )
{
}
CardanConstraint::CardanConstraint( const CardanConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axisA( rhs._axisA ),
    _axisB( rhs._axisB )
{
}
CardanConstraint::~CardanConstraint()
{
}

btUniversalConstraint* CardanConstraint::getAsBtUniversal() const
{
    return( static_cast< btUniversalConstraint* >( getConstraint() ) );
}

void CardanConstraint::setAxisA( const osg::Vec3& axisA )
{
    _axisA = axisA;
    setDirty( true );
}
void CardanConstraint::setAxisB( const osg::Vec3& axisB )
{
    _axisB = axisB;
    setDirty( true );
}

bool CardanConstraint::operator==( const CardanConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool CardanConstraint::operator!=( const CardanConstraint& rhs ) const
{
    return(
        ( _axisA != rhs._axisA ) ||
        ( _axisB != rhs._axisB ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void CardanConstraint::createConstraint()
{
    // TBD make sure axisA and axisB are at 90 degree angles.
}
    



BallAndSocketConstraint::BallAndSocketConstraint()
  : Constraint()
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform, const osg::Vec3& wcPoint )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _point( wcPoint )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& wcPoint )
  : Constraint( rbA, rbAXform ),
    _point( wcPoint )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( const BallAndSocketConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _point( rhs._point )
{
}
BallAndSocketConstraint::~BallAndSocketConstraint()
{
}

btPoint2PointConstraint* BallAndSocketConstraint::getAsBtPoint2Point() const
{
    return( static_cast< btPoint2PointConstraint* >( getConstraint() ) );
}

void BallAndSocketConstraint::setPoint( const osg::Vec3& wcPoint )
{
    _point = wcPoint;
    setDirty();
}

bool BallAndSocketConstraint::operator==( const BallAndSocketConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool BallAndSocketConstraint::operator!=( const BallAndSocketConstraint& rhs ) const
{
    return(
        ( _point != rhs._point ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void BallAndSocketConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint )
        delete _constraint;


    // Compute a matrix that transforms the world coord point into
    // A's collision shape local coordinates.
    //
    //   1. Handle A's center of mass offset.
    osg::Vec3 aCom;
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "BallAndSocketConstraint: Invalid MotionState." << std::endl;
        return;
    }
    aCom = motion->getCenterOfMass();
    //
    //   2. The final transform matrix.
    osg::Matrix rbAMatrix = osg::Matrix::inverse( osg::Matrix::translate( aCom ) * _rbAXform );

    // And now compute the WC point in rbA space:
    const btVector3 aPt = osgbCollision::asBtVector3( _point * rbAMatrix );


    // Compute a matrix that transforms the world coord point into
    // B's collision shape local coordinates.
    osg::Matrix rbBMatrix;
    if( _rbB != NULL )
    {
        //
        //   1. Handle B's center of mass offset.
        osg::Vec3 bCom;
        motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "BallAndSocketConstraint: Invalid MotionState." << std::endl;
            return;
        }
        bCom = motion->getCenterOfMass();
        //
        //   2. The final transform matrix.
        rbBMatrix = osg::Matrix::inverse( osg::Matrix::translate( bCom ) * _rbBXform );
    }

    // And now compute the WC point in rbB space:
    const btVector3 bPt = osgbCollision::asBtVector3( _point * rbBMatrix );


    btPoint2PointConstraint* cons;
    if( _rbB != NULL )
        cons = new btPoint2PointConstraint( *_rbA, *_rbB, aPt, bPt );
    else
        cons = new btPoint2PointConstraint( *_rbA, aPt );
    _constraint = cons;

    setDirty( false );
}




RagdollConstraint::RagdollConstraint()
  : Constraint()
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& wcPoint, const osg::Vec3& wcAxis, const double angleRadians )
  : Constraint( rbA, rbAXform ),
    _point( wcPoint ),
    _axis( wcAxis ),
    _angle( angleRadians )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& wcPoint, const osg::Vec3& wcAxis, const double angleRadians )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _point( wcPoint ),
    _axis( wcAxis ),
    _angle( angleRadians )
{
}
RagdollConstraint::RagdollConstraint( const RagdollConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _point( rhs._point ),
    _axis( rhs._axis ),
    _angle( rhs._angle )
{
}
RagdollConstraint::~RagdollConstraint()
{
}

btConeTwistConstraint* RagdollConstraint::getAsBtConeTwist() const
{
    return( static_cast< btConeTwistConstraint* >( getConstraint() ) );
}

void RagdollConstraint::setPoint( const osg::Vec3& wcPoint )
{
    _point = wcPoint;
    setDirty( true );
}
void RagdollConstraint::setAxis( const osg::Vec3& wcAxis )
{
    _axis = wcAxis;
    setDirty( true );
}
void RagdollConstraint::setAngle( const double angleRadians )
{
    _angle = angleRadians;
    setDirty( true );
}

bool RagdollConstraint::operator==( const RagdollConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool RagdollConstraint::operator!=( const RagdollConstraint& rhs ) const
{
    return(
        ( _point != rhs._point ) ||
        ( _axis != rhs._axis ) ||
        ( _angle != rhs._angle ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void RagdollConstraint::createConstraint()
{
}




WheelSuspensionConstraint::WheelSuspensionConstraint()
  : Constraint()
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& springAxis, const osg::Vec3& axleAxis )
  : Constraint( rbA, rbAXform ),
    _springAxis( springAxis ),
    _axleAxis( axleAxis )
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& springAxis, const osg::Vec3& axleAxis )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _springAxis( springAxis ),
    _axleAxis( axleAxis )
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( const WheelSuspensionConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _springAxis( rhs._springAxis ),
    _axleAxis( rhs._axleAxis )
{
}
WheelSuspensionConstraint::~WheelSuspensionConstraint()
{
}

btHinge2Constraint* WheelSuspensionConstraint::getAsBtHinge2() const
{
    return( static_cast< btHinge2Constraint* >( getConstraint() ) );
}

void WheelSuspensionConstraint::setSpringAxis( const osg::Vec3& springAxis )
{
    _springAxis = springAxis;
    setDirty( true );
}
void WheelSuspensionConstraint::setAxleAxis( const osg::Vec3& axleAxis )
{
    _axleAxis = axleAxis;
    setDirty( true );
}

bool WheelSuspensionConstraint::operator==( const WheelSuspensionConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool WheelSuspensionConstraint::operator!=( const WheelSuspensionConstraint& rhs ) const
{
    return(
        ( _springAxis != rhs._springAxis ) ||
        ( _axleAxis != rhs._axleAxis ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void WheelSuspensionConstraint::createConstraint()
{
    // TBD make sure spring and axle are at 90 degree angles.
}



// osgbDynamics
}
