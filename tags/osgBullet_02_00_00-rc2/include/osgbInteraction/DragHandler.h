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

#ifndef __OSGBINTERACTION_DRAG_HANDLER_H__
#define __OSGBINTERACTION_DRAG_HANDLER_H__ 1


#include <osgbInteraction/Export.h>
#include <osgGA/GUIEventHandler>
#include <osgbDynamics/MotionState.h>
#include <btBulletDynamicsCommon.h>
#include <osg/Vec4>


// Forward
namespace osg {
    class Camera;
}

namespace osgbDynamics {
    class PhysicsThread;
}


namespace osgbInteraction
{


/** \class DragHandler DragHandler.h <osgbInteraction\DragHandler.h>
\brief And event handler for selecting and dragging rigid bodies.

TBD full descrip.
*/
class OSGBINTERACTION_EXPORT DragHandler : public osgGA::GUIEventHandler
{
public:
    /** \brief Constructor.
    \param dw
    \param scene
    */
    DragHandler( btDynamicsWorld* dw, osg::Camera* scene );

    /** \brief Handle events

    Controls:
    \li ctrl-left-mouse Select and drag a rigid body.
    */
    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );

    void setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt );

protected:
    ~DragHandler();

    /**
    \param wx Normalized (-1.0 to 1.0) x mouse position
    \param wy Normalized (-1.0 to 1.0) Y mouse position
    */
    bool pick( float wx, float wy );

    btDynamicsWorld* _dw;
    osg::ref_ptr< osg::Camera > _scene;

    btPoint2PointConstraint* _constraint;
    osgbDynamics::MotionState* _constrainedMotionState;
    osg::Vec4 _dragPlane;

    osgbDynamics::PhysicsThread* _pt;
};


// osgbInteraction
}


// __OSGBINTERACTION_DRAG_HANDLER_H__
#endif