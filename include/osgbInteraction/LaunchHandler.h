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

#ifndef __OSGBINTERACTION_LAUNCH_HANDLER_H__
#define __OSGBINTERACTION_LAUNCH_HANDLER_H__ 1


#include <osgbInteraction/Export.h>
#include <osgGA/GUIEventHandler>
#include <btBulletDynamicsCommon.h>
#include <osg/ref_ptr>
#include <list>

// Forward
namespace osg {
    class Node;
    class Group;
    class Camera;
}


namespace osgbInteraction
{


/** \class LaunchHandler LaunchHandler.h <osgbInteraction\LaunchHandler.h>
\brief An event handler to support throwing objects into a dynamics simulation.

TBD full descrip.
*/
class OSGBINTERACTION_EXPORT LaunchHandler : public osgGA::GUIEventHandler
{
public:
    LaunchHandler( btDynamicsWorld* dw, osg::Group* attachPoint, osg::Camera* camera=NULL );

    void setLaunchModel( osg::Node* model, btCollisionShape* shape=NULL );

    /** \brief Access the initial launch velocity.

    Units are in physics simultation units (e.g., m/sec^2). Default is 10.0. */
    void setInitialVelocity( double velocity ) { _initialVelocity = velocity; }
    double getInitialVelocity() const { return( _initialVelocity ); }

    void setCollisionFlags( short group, short mask ) { _group = group; _mask = mask; }

    void setCamera( osg::Camera* camera ) { _camera = camera; }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& );

    /** \brief Remove all launched models from the scene graph and physics simultation.
    */
    void reset();

protected:
    ~LaunchHandler();

    btDynamicsWorld* _dw;
    osg::ref_ptr< osg::Group > _attachPoint;
    osg::ref_ptr< osg::Camera > _camera;

    osg::ref_ptr< osg::Node > _launchModel;
    btCollisionShape* _launchCollisionShape;
    double _initialVelocity;
    short _group, _mask;

    typedef std::list< osg::ref_ptr< osg::Node > > NodeList;
    NodeList _nodeList;
};


// osgbInteraction
}


// __OSGBINTERACTION_LAUNCH_HANDLER_H__
#endif
