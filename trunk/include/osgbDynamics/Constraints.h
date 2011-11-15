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

#ifndef __OSGBDYNAMICS_CONSTRAINTS_H__
#define __OSGBDYNAMICS_CONSTRAINTS_H__ 1


#include <osgbDynamics/Export.h>

#include <osg/Object>
#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec2>
#include <osg/ref_ptr>

#include <btBulletDynamicsCommon.h>

#include <list>
#include <string>



namespace osgbDynamics
{


/** \defgroup constraintsupport Constraint Support
\brief Convenience objects to support and enhance Bullet's constraint objects.

These objects are designed to make it easier for OSG developers to work with
Bullet constraints. The primary features are:

\li Each class is an osg::Object with dot OSG file format support.
\li Initial rigid body transforms are osg::Matrix objects representing the initial
transform of the object in OSG coordinate space.
\li The classes provide full support for collision shapes with non-origin center of
mass and non-unit scaling.

Note that the calling code is responsible for deleting the Bullet constraing.
See Constraint::~Constraint().
*/
/**@{*/

/** \class Constraint Constraint.h <osgbDynamics/Constraint.h>
\brief Base Constraint class with support for rigid bodies and transforms, lazy
Bullet constraint creation, constraint access, and typecasting.
*/
class OSGBDYNAMICS_EXPORT Constraint : public osg::Object
{
public:
    Constraint();
    Constraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    Constraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB=NULL, const osg::Matrix& rbBXform=osg::Matrix::identity() );
    Constraint( const Constraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,Constraint);

    /** \brief Get the constraint as a btTypedConstraint*.
    
    Note: This function will perform a const_cast if necessary to create the constraint
    (if it's dirty or doesn't yet exist). */
    virtual btTypedConstraint* getConstraint() const;
    /** \brief If the derived subclass uses a btConeTwistConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btConeTwistConstraint* getAsBtConeTwist() const { return( NULL ); }
    /** \brief If the derived subclass uses a btContactConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btContactConstraint* getAsBtContact() const { return( NULL ); }
    /** \brief If the derived subclass uses a btGeneric6DofConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const { return( NULL ); }
    /** \brief If the derived subclass uses a btGeneric6DofSpringConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const { return( NULL ); }
    /** \brief If the derived subclass uses a btHingeConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btHingeConstraint* getAsBtHinge() const { return( NULL ); }
    /** \brief If the derived subclass uses a btHinge2Constraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btHinge2Constraint* getAsBtHinge2() const { return( NULL ); }
    /** \brief If the derived subclass uses a btPoint2PointConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btPoint2PointConstraint* getAsBtPoint2Point() const { return( NULL ); }
    /** \brief If the derived subclass uses a btSliderConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btSliderConstraint* getAsBtSlider() const { return( NULL ); }
    /** \brief If the derived subclass uses a btUniversalConstraint* internally, the subclass
    will override this function to return non-NULL. The getConstraint() const_cast note also applies. */
    virtual btUniversalConstraint* getAsBtUniversal() const { return( NULL ); }

    /** \brief Access the Bullet rigid body (or bodies).

    If only one rigid body is specified, it is constrained to the world.
    Otherwise, both rigid bodies are constrained together. */
    void setRigidBodies( btRigidBody* rbA, btRigidBody* rbB=NULL );
    void getRigidBodies( btRigidBody* rbA, btRigidBody* rbB )
    {
        rbA = _rbA; rbB = _rbB;
    }

    /** \brief Specify the initial OSG transform of the subgraph corresponding to rigid body A.
    
    setAXform() dirties the Constraint, so the next call to getConstraint() will
    delete the current constraint (if it exists) and create a new one. */
    void setAXform( const osg::Matrix& rbAXform );
    osg::Matrix getAXform() const
    {
        return( _rbAXform );
    }

    /** \brief Specify the initial OSG transform of the subgraph corresponding to rigid body B.

    setBXform() dirties the Constraint, so the next call to getConstraint() will
    delete the current constraint (if it exists) and create a new one. */
    void setBXform( const osg::Matrix& rbBXform );
    osg::Matrix getBXform() const
    {
        return( _rbBXform );
    }

    /** \brief Set the dirty bit to indicate the constraint paramters have changed.

    This function implements a lazy constraint creation mechanism so that multiple
    parameters may be changed without deleting and re-creating the constraint multiple
    times. The getConstraint() function will delete (if necessary) and create the new
    Bullet constraint if \c _dirty is true. \c _dirty is initially true for all new
    Constraint and Constraing-derived objects. */
    void setDirty( bool dirty=true )
    {
        _dirty = dirty;
    }
    bool getDirty() const
    {
        return( _dirty );
    }

    /** Return true if both rigid body transform member variables are
    equal to the right-hand-side transforms. This function does not compare
    rigid body addresses. */
    virtual bool operator==( const Constraint& rhs ) const;
    /** Return true if either rigid body transform member variable differs
    from the right-hand-side transforms. This function does not compare
    rigid body addresses. */
    virtual bool operator!=( const Constraint& rhs ) const;

protected:
    /** \brief Destructor.
    Note that deleting the constraint is up to the calling code. For example:
    \code
    delete osgbDynamics::Constraint::getConstraint();
    osgbDynamics::Constraint::setDirty();
    \endcode
    */
    virtual ~Constraint();

    /** \brief Create the constraint, if it doesn't exist, or delete it and re-create it if
    the \c _dirty flag indicates that the parameters have changed.

    Note: Use of META_Object inhibits making this function pure virtual.
    Constraint::getConstraing() is a no-op, and all derived classes must override this function
    to implement Bullet constraint creation. */
    virtual void createConstraint() {}

    btTypedConstraint* _constraint;
    bool _dirty;

    btRigidBody* _rbA;
    btRigidBody* _rbB;
    osg::Matrix _rbAXform;
    osg::Matrix _rbBXform;
};
typedef std::list< osg::ref_ptr< Constraint > > ConstraintList;


/** \class SliderConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief Creates a constraint from an axis and movement limits on that axis.
*/
class OSGBDYNAMICS_EXPORT SliderConstraint : public Constraint
{
public:
    SliderConstraint();
    SliderConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit );
    SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit );
    SliderConstraint( const SliderConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,SliderConstraint);

    virtual btSliderConstraint* getAsBtSlider() const;

    /** \brief Specify the slider axis in the world coordinate space.

    This is the axis along which the two constrained bodies are allowed to move
    relative to each other. In the common scenario where one rigid body is fixed,
    the other rigid body moves along this axis. If \c _rbB is NULL, \c _rbA
    moves along this axis. */
    void setAxis( const osg::Vec3& axis );
    osg::Vec3 getAxis() const
    {
        return( _axis );
    }

    /** \brief Specify movement limits along \c _sliderAxisInA.

    The limit values are in world coordinates and relative to the initial transforms
    \c _rbAXform and \c _rbBXform. */
    void setLimit( const osg::Vec2& limit );
    osg::Vec2 getLimit() const
    {
        return( _slideLimit );
    }

    /** Return true if the axis and limit member variables, and base class, are
    equal to the right-hand-side axis, limit, and base class. */
    virtual bool operator==( const SliderConstraint& rhs ) const;
    /** Return true if the axis or limit member variables, or base class, differ
    from the right-hand-side axis, limit, or base class. */
    virtual bool operator!=( const SliderConstraint& rhs ) const;

protected:
    virtual ~SliderConstraint();

    virtual void createConstraint();

    osg::Vec3 _axis;
    osg::Vec2 _slideLimit;
};


/** \class TwistSliderConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief A SliderConstraint that allows rotation around the slide axis.
*/
class OSGBDYNAMICS_EXPORT TwistSliderConstraint : public SliderConstraint
{
public:
    TwistSliderConstraint();
    TwistSliderConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    TwistSliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit );
    TwistSliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit );
    TwistSliderConstraint( const TwistSliderConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

protected:
    virtual ~TwistSliderConstraint();

    virtual void createConstraint();
};


/** \class LinearSpringConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT LinearSpringConstraint : public Constraint
{
public:
    LinearSpringConstraint();

protected:
    virtual ~LinearSpringConstraint();
};


/** \class AngleSpringConstraint AngleSpringConstraintConstraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT AngleSpringConstraint : public Constraint
{
public:
    AngleSpringConstraint();

protected:
    virtual ~AngleSpringConstraint();
};


/** \class LinearAngleSpringConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT LinearAngleSpringConstraint : public Constraint
{
public:
    LinearAngleSpringConstraint();

protected:
    virtual ~LinearAngleSpringConstraint();
};


/** \class FixedConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT FixedConstraint : public Constraint
{
public:
    FixedConstraint();
    FixedConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    FixedConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB=NULL, const osg::Matrix& rbBXform=osg::Matrix::identity() );
    FixedConstraint( const FixedConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,FixedConstraint);

    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const;

protected:
    virtual ~FixedConstraint();

    void createConstraint();
};


/** \class PlanarConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT PlanarConstraint : public Constraint
{
public:
    PlanarConstraint();
    PlanarConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL,
            const osg::Vec2& loLimit=osg::Vec2(0.,0.),
            const osg::Vec2& hiLimit=osg::Vec2(0.,0.),
            const osg::Matrix& orient=osg::Matrix::identity() );
    PlanarConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB=NULL, const osg::Matrix& rbBXform=osg::Matrix::identity(),
            const osg::Vec2& loLimit=osg::Vec2(0.,0.),
            const osg::Vec2& hiLimit=osg::Vec2(0.,0.),
            const osg::Matrix& orient=osg::Matrix::identity() );
    PlanarConstraint( const PlanarConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,PlanarConstraint);

    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const;

    void setLowLimit( const osg::Vec2& loLimit );
    void setHighLimit( const osg::Vec2& hiLimit );
    osg::Vec2 getLowLimit() const
    {
        return( _loLimit );
    }
    osg::Vec2 getHighLimit() const
    {
        return( _hiLimit );
    }

    void setOrientation( const osg::Matrix& orient );
    osg::Matrix getOrientation() const
    {
        return( _orient );
    }

    /** Return true if the lower and upper limit member variables, and base class, are
    equal to the right-hand-side lower and upper limits and base class. */
    virtual bool operator==( const PlanarConstraint& rhs ) const;
    /** Return true if the lower and upper limit member variables, or base class, differ
    from the right-hand-side lower and upper limits or base class. */
    virtual bool operator!=( const PlanarConstraint& rhs ) const;

protected:
    virtual ~PlanarConstraint();

    void createConstraint();

    osg::Vec2 _loLimit, _hiLimit;
    osg::Matrix _orient;
};


/** \class BoxConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT BoxConstraint : public Constraint
{
public:
    BoxConstraint();
    BoxConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL,
            const osg::Vec3& loLimit=osg::Vec3(0.,0.,0.),
            const osg::Vec3& hiLimit=osg::Vec3(0.,0.,0.),
            const osg::Matrix& orient=osg::Matrix::identity() );
    BoxConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB=NULL, const osg::Matrix& rbBXform=osg::Matrix::identity(),
            const osg::Vec3& loLimit=osg::Vec3(0.,0.,0.),
            const osg::Vec3& hiLimit=osg::Vec3(0.,0.,0.),
            const osg::Matrix& orient=osg::Matrix::identity() );
    BoxConstraint( const BoxConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,BoxConstraint);

    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const;

    void setLowLimit( const osg::Vec3& loLimit );
    void setHighLimit( const osg::Vec3& hiLimit );
    osg::Vec3 getLowLimit() const
    {
        return( _loLimit );
    }
    osg::Vec3 getHighLimit() const
    {
        return( _hiLimit );
    }

    void setOrientation( const osg::Matrix& orient );
    osg::Matrix getOrientation() const
    {
        return( _orient );
    }

    /** Return true if the lower and upper limit member variables, and base class, are
    equal to the right-hand-side lower and upper limits and base class. */
    virtual bool operator==( const BoxConstraint& rhs ) const;
    /** Return true if the lower and upper limit member variables, or base class, differ
    from the right-hand-side lower and upper limits or base class. */
    virtual bool operator!=( const BoxConstraint& rhs ) const;

protected:
    virtual ~BoxConstraint();

    void createConstraint();

    osg::Vec3 _loLimit, _hiLimit;
    osg::Matrix _orient;
};


/** \class HingeConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT HingeConstraint : public Constraint
{
public:
    HingeConstraint();

protected:
    virtual ~HingeConstraint();
};


/** \class CardanConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT CardanConstraint : public Constraint
{
public:
    CardanConstraint();

protected:
    virtual ~CardanConstraint();
};


/** \class BallAndSocketConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief Constrains two rigid bodies at the same world coordinate point.
*/
class OSGBDYNAMICS_EXPORT BallAndSocketConstraint : public Constraint
{
public:
    BallAndSocketConstraint();
    BallAndSocketConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& wcPoint );
    BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& wcPoint );
    BallAndSocketConstraint( const BallAndSocketConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,BallAndSocketConstraint);

    virtual btPoint2PointConstraint* getAsBtPoint2Point() const;

    /** \brief Specify the common point in world coordinates.

    The rbA and rbB transforms are used to convert this point into local coordinates,
    which are then passed into the btPoint2PointConstraint constructor.
    */
    void setPoint( const osg::Vec3& point );
    osg::Vec3 getPoint() const
    {
        return( _point );
    }

    /** Return true if the point member variables and base class are
    equal to the right-hand-side point and base class. */
    virtual bool operator==( const BallAndSocketConstraint& rhs ) const;
    /** Return true if the point member variable or base class differ
    from the right-hand-side point or base class. */
    virtual bool operator!=( const BallAndSocketConstraint& rhs ) const;

protected:
    virtual ~BallAndSocketConstraint();

    virtual void createConstraint();

    osg::Vec3 _point;
};


/** \class RagdollConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT RagdollConstraint : public Constraint
{
public:
    RagdollConstraint();

protected:
    virtual ~RagdollConstraint();
};


/** \class WheelSuspensionConstraint Constraint.h <osgbDynamics/Constraint.h>
\brief TBD
*/
class OSGBDYNAMICS_EXPORT WheelSuspensionConstraint : public Constraint
{
public:
    WheelSuspensionConstraint();

protected:
    virtual ~WheelSuspensionConstraint();
};


/**@}*/


// osgbDynamics
}


// __OSGBDYNAMICS_CONSTRAINTS_H__
#endif
