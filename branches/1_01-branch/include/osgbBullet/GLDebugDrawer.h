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

#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include <LinearMath/btIDebugDraw.h>
#include <osgbBullet/Chart.h>

#include <osgbBullet/Export.h>

#include <osg/Geometry>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Camera>

namespace osgText {
    class Text;
};


namespace osgbBullet
{

class OSGBBULLET_EXPORT GLDebugDrawer : public btIDebugDraw
{
public:
    GLDebugDrawer();
    virtual ~GLDebugDrawer();

    osg::Node* getSceneGraph();

    void setEnabled( bool enable );
    bool getEnabled() const;

    virtual void	drawLine( const btVector3& from,const btVector3& to,const btVector3& color );
    virtual void	drawSphere( const btVector3& p, btScalar radius, const btVector3& color );

    virtual void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);
    virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);
    virtual void	reportErrorWarning(const char* warningString);
    virtual void	draw3dText(const btVector3& location,const char* textString);

    virtual void	setDebugMode(int debugMode);
    virtual int		getDebugMode() const;

    void EndDraw();
    void BeginDraw();

    void setTextSize( const float size );
    float getTextSize() const;

private:
    osgText::Text* initText();

	int _debugMode;
    bool _enabled;
    bool _active;
    float _textSize;

    osg::ref_ptr< osg::Group > _group;

    // Points, lines, triangles, and text
    osg::ref_ptr< osg::Geode > _geode;
    osg::ref_ptr< osg::Geometry > _ptGeom;
    osg::ref_ptr< osg::Geometry > _lnGeom;
    osg::ref_ptr< osg::Geometry > _triGeom;

    typedef std::vector< osg::ref_ptr< osgText::Text > > TextVec;
    TextVec _textVec;
    unsigned int _textStrings;

    osg::Vec3Array* _ptVerts;
    osg::Vec4Array* _ptColors;

    osg::Vec3Array* _lnVerts;
    osg::Vec4Array* _lnColors;

    osg::Vec3Array* _triVerts;
    osg::Vec4Array* _triColors;

    // HUD display
    osg::ref_ptr< osg::Camera > _hudCam;

    osg::ref_ptr< osgbBullet::Chart > _chart;
    int _frame;
    int _contacts;
};

}

#endif//GL_DEBUG_DRAWER_H
