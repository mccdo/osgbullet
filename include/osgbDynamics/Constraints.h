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

\test testconstraint
*/
/**@{*/

/** \class Constraint Constraints.h <osgbDynamics/Constraints.h>
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
    void getRigidBodies( btRigidBody*& rbA, btRigidBody*& rbB )
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


/** \class SliderConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief Creates a constraint from an axis and movement limits on that axis.

This class uses btSliderConstraint internally. Access the Bullet constraint
directly with getAsBtSlider().
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


/** \class TwistSliderConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief A SliderConstraint that allows rotation around the slide axis.

Because this class derives from SliderConstraint, it uses btSliderConstraint
internally. Access the Bullet constraint directly with
SliderConstraint::getAsBtSlider().
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
    META_Object(osgbDynamics,TwistSliderConstraint);

protected:
    virtual ~TwistSliderConstraint();

    virtual void createConstraint();
};


/** \class LinearSpringConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btGeneric6DofSpringConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6DofSpring().
*/
class OSGBDYNAMICS_EXPORT LinearSpringConstraint : public Constraint
{
public:
    LinearSpringConstraint();
    LinearSpringConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    LinearSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform );
    LinearSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform );
    LinearSpringConstraint( const LinearSpringConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,LinearSpringConstraint);

    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const;

protected:
    virtual ~LinearSpringConstraint();

    virtual void createConstraint();
};


/** \class AngleSpringConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btGeneric6DofSpringConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6DofSpring().
*/
class OSGBDYNAMICS_EXPORT AngleSpringConstraint : public Constraint
{
public:
    AngleSpringConstraint();
    AngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    AngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform );
    AngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform );
    AngleSpringConstraint( const AngleSpringConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,AngleSpringConstraint);

    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const;

protected:
    virtual ~AngleSpringConstraint();

    virtual void createConstraint();
};


/** \class LinearAngleSpringConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btGeneric6DofSpringConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6DofSpring().
*/
class OSGBDYNAMICS_EXPORT LinearAngleSpringConstraint : public Constraint
{
public:
    LinearAngleSpringConstraint();
    LinearAngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    LinearAngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform );
    LinearAngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform );
    LinearAngleSpringConstraint( const LinearAngleSpringConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,LinearAngleSpringConstraint);

    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const;

protected:
    virtual ~LinearAngleSpringConstraint();

    virtual void createConstraint();
};


/** \class FixedConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btGeneric6DofConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6Dof().
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

    virtual void createConstraint();
};


/** \class PlanarConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief Allows bodies to move relative to each other in a plane.

The user can specify the orientation (\c _orient) of the plane using setOrient(). \c _orient
matrix determines the plane orientation based on the following pseudocode:

\code
  if _orient is identity
    the plane is in B'a coord space.
    if B is NULL
      the plane is in the world coord space.
  else
    the plane coord space is B's coord space, multiplied
            by _orient.
    if B is NULL
      the plane coord space is defined solely by _orient.
\endcode
"Coord space" above refers to orientation only. The origin is always based
on the initial local-to-world transform(s).

This class uses btGeneric6DofConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6Dof().
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

    /** \brief Specify the orienation of the constrained axes.

    Note that this class ignores any translation component in \c orient.
    */
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

    virtual void createConstraint();

    osg::Vec2 _loLimit, _hiLimit;
    osg::Matrix _orient;
};


/** \class BoxConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

WARNING Not yet fully functional. See PlanarConstraint.

This class uses btGeneric6DofConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6Dof().
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
    BoxConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
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

    virtual void createConstraint();

    osg::Vec3 _loLimit, _hiLimit;
    osg::Matrix _orient;

private:
    /** \brief Shared reference frame computation code for both Planar and
    Box constraints. Access to Planar is allowed via "friend" declarative. */
    static void internalPlanarBoxFrameComputation(
        btTransform& aFrame, btTransform& bFrame,
        Constraint* cons, const osg::Matrix& orient );
    friend class PlanarConstraint;
};


/** \class HingeConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btHingeConstraint internally. Access the Bullet constraint
directly with getAsBtHinge().
*/
class OSGBDYNAMICS_EXPORT HingeConstraint : public Constraint
{
public:
    HingeConstraint();
    HingeConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL,
            const osg::Vec3& axis=osg::Vec3(0.,0.,1.),
            const osg::Vec3& pivotPoint=osg::Vec3(0.,0.,0.),
            const osg::Vec2& limit=osg::Vec2(-osg::PI,osg::PI) );
    HingeConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB=NULL, const osg::Matrix& rbBXform=osg::Matrix::identity(),
            const osg::Vec3& axis=osg::Vec3(0.,0.,1.),
            const osg::Vec3& pivotPoint=osg::Vec3(0.,0.,0.),
            const osg::Vec2& limit=osg::Vec2(-osg::PI,osg::PI) );
    HingeConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& axis=osg::Vec3(0.,0.,1.),
            const osg::Vec3& pivotPoint=osg::Vec3(0.,0.,0.),
            const osg::Vec2& limit=osg::Vec2(-osg::PI,osg::PI) );
    HingeConstraint( const HingeConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,HingeConstraint);

    virtual btHingeConstraint* getAsBtHinge() const;

    void setAxis( const osg::Vec3& axis );
    osg::Vec3 getAxis() const
    {
        return( _axis );
    }

    void setPivotPoint( const osg::Vec3& wcPoint );
    osg::Vec3 getPivotPoint() const
    {
        return( _pivotPoint );
    }

    void setLimit( const osg::Vec2& limit );
    osg::Vec2 getLimit() const
    {
        return( _limit );
    }

    /** Return true if the axis, pivot point, and limit member variables, and base class, are
    equal to the right-hand-side axis, pivot point, limit, and base class. */
    virtual bool operator==( const HingeConstraint& rhs ) const;
    /** Return true if the axis, pivot point, or limit member variables, or base class, differ
    from the right-hand-side axis, pivot point, limit, or base class. */
    virtual bool operator!=( const HingeConstraint& rhs ) const;

protected:
    virtual ~HingeConstraint();

    virtual void createConstraint();

    osg::Vec3 _axis;
    osg::Vec3 _pivotPoint;
    osg::Vec2 _limit;
};


/** \class CardanConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btUniversalConstraint internally. Access the Bullet constraint
directly with getAsBtUniversal().
*/
class OSGBDYNAMICS_EXPORT CardanConstraint : public Constraint
{
public:
    CardanConstraint();
    CardanConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    CardanConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& axisA=osg::Vec3(0.,1.,0.),
            const osg::Vec3& axisB=osg::Vec3(1.,0.,0.) );
    CardanConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& axisA=osg::Vec3(0.,1.,0.),
            const osg::Vec3& axisB=osg::Vec3(1.,0.,0.) );
    CardanConstraint( const CardanConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,CardanConstraint);

    virtual btUniversalConstraint* getAsBtUniversal() const;

    /**
    */
    void setAxisA( const osg::Vec3& axisA );
    osg::Vec3 getAxisA() const
    {
        return( _axisA );
    }

    /**
    */
    void setAxisB( const osg::Vec3& axisB );
    osg::Vec3 getAxisB() const
    {
        return( _axisB );
    }

    /** Return true if the axes member variables, and base class, are
    equal to the right-hand-side axes and base class. */
    virtual bool operator==( const CardanConstraint& rhs ) const;
    /** Return true if the axes member variables, or base class, differ
    from the right-hand-side axes or base class. */
    virtual bool operator!=( const CardanConstraint& rhs ) const;

protected:
    virtual ~CardanConstraint();

    virtual void createConstraint();

    osg::Vec3 _axisA;
    osg::Vec3 _axisB;
};


/** \class BallAndSocketConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief Constrains two rigid bodies at the same world coordinate point.

This class uses btPoint2PointConstraint internally. Access the Bullet constraint
directly with getAsBtPoint2Point().
*/
class OSGBDYNAMICS_EXPORT BallAndSocketConstraint : public Constraint
{
public:
    BallAndSocketConstraint();
    BallAndSocketConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& wcPoint=osg::Vec3(0.,0.,0.) );
    BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& wcPoint=osg::Vec3(0.,0.,0.) );
    BallAndSocketConstraint( const BallAndSocketConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,BallAndSocketConstraint);

    virtual btPoint2PointConstraint* getAsBtPoint2Point() const;

    /** \brief Specify the common point in world coordinates.

    The rbA and rbB transforms are used to convert this point into local coordinates,
    which are then passed into the btPoint2PointConstraint constructor.
    */
    void setPoint( const osg::Vec3& wcPoint );
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


/** \class RagdollConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btConeTwistConstraint internally. Access the Bullet constraint
directly with getAsBtConeTwist().
*/
class OSGBDYNAMICS_EXPORT RagdollConstraint : public Constraint
{
public:
    RagdollConstraint();
    RagdollConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    RagdollConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& wcPoint=osg::Vec3(0.,0.,0),
            const osg::Vec3& wcAxis=osg::Vec3(1.,0.,0),
            const double angleRadians=0. );
    RagdollConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& wcPoint=osg::Vec3(0.,0.,0),
            const osg::Vec3& wcAxis=osg::Vec3(1.,0.,0),
            const double angleRadians=0. );
    RagdollConstraint( const RagdollConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,RagdollConstraint);

    virtual btConeTwistConstraint* getAsBtConeTwist() const;

    /** \brief Specify the common point in world coordinates.
    */
    void setPoint( const osg::Vec3& wcPoint );
    osg::Vec3 getPoint() const
    {
        return( _point );
    }
    /** \brief Specify an axis corresponding to the center of the cone.
    */
    void setAxis( const osg::Vec3& wcAxis );
    osg::Vec3 getAxis() const
    {
        return( _axis );
    }
    /** \brief Specify the cone spread angle in radians.
    */
    void setAngle( const double angleRadians );
    double getAngle() const
    {
        return( _angle );
    }

    /** Return true if the point, axis, and angle member variables, and base class, are
    equal to the right-hand-side point, axis, angle, and base class. */
    virtual bool operator==( const RagdollConstraint& rhs ) const;
    /** Return true if the point, axis, or angle member variables, or base class, differ
    from the right-hand-side point, axis, angle, or base class. */
    virtual bool operator!=( const RagdollConstraint& rhs ) const;

protected:
    virtual ~RagdollConstraint();

    virtual void createConstraint();

    osg::Vec3 _point;
    osg::Vec3 _axis;
    double _angle;
};


/** \class WheelSuspensionConstraint Constraints.h <osgbDynamics/Constraints.h>
\brief TBD

This class uses btHinge2Constraint internally. Access the Bullet constraint
directly with getAsBtHinge2().
*/
class OSGBDYNAMICS_EXPORT WheelSuspensionConstraint : public Constraint
{
public:
    WheelSuspensionConstraint();
    WheelSuspensionConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    WheelSuspensionConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& springAxis=osg::Vec3(0.,1.,0),
            const osg::Vec3& axleAxis=osg::Vec3(1.,0.,0) );
    WheelSuspensionConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& springAxis=osg::Vec3(0.,1.,0),
            const osg::Vec3& axleAxis=osg::Vec3(1.,0.,0) );
    WheelSuspensionConstraint( const WheelSuspensionConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,WheelSuspensionConstraint);

    virtual btHinge2Constraint* getAsBtHinge2() const;

    /** \brief Specify the spring axis in world space.

    The spring axis allows +/- motion along the axis, as well
    as arbitrary rotation.

    The spring and axle axes must be orthogonal. This class resolves
    any non-orthogonality issues when it creates the Bullet constraing
    (in createConstraing()).
    */
    void setSpringAxis( const osg::Vec3& springAxis );
    osg::Vec3 getSpringAxis() const
    {
        return( _springAxis );
    }
    /** \brief Specify the axle axis in world space.

    The spring and axle axes must be orthogonal. This class resolves
    any non-orthogonality issues when it creates the Bullet constraing
    (in createConstraing()).
    */
    void setAxleAxis( const osg::Vec3& axleAxis );
    osg::Vec3 getAxleAxis() const
    {
        return( _axleAxis );
    }

    /** Return true if the spring and axle axes member variables, and base class, are
    equal to the right-hand-side spring and axle axes and base class. */
    virtual bool operator==( const WheelSuspensionConstraint& rhs ) const;
    /** Return true if the spring and axle axes member variables, or base class, differ
    from the right-hand-side spring and axle axes, or base class. */
    virtual bool operator!=( const WheelSuspensionConstraint& rhs ) const;

protected:
    virtual ~WheelSuspensionConstraint();

    virtual void createConstraint();

    osg::Vec3 _springAxis;
    osg::Vec3 _axleAxis;
};


/**@}*/


// osgbDynamics
}


// __OSGBDYNAMICS_CONSTRAINTS_H__
#endif
