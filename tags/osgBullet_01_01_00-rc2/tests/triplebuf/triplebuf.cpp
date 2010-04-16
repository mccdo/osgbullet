// Copyright (c) 2009 Blue Newt Software LLC. All rights reserved.

#include <osgbBullet/TripleBuffer.h>
#include <OpenThreads/Thread>
#include <osg/Timer>
#include <iostream>


class WriteThread : public OpenThreads::Thread
{
public:
    WriteThread( osgbBullet::TripleBuffer* tb )
      : _tb( tb ),
        _count( 0 )
    {}

    virtual void run()
    {
        while( !testCancel() )
        {
            char* addr = _tb->beginWrite();
            OpenThreads::Thread::microSleep( 10000 );
            unsigned int* iAddr = reinterpret_cast< unsigned int* >( addr );
            *iAddr = _count++;
            _tb->endWrite();
        }
    }

protected:
    osgbBullet::TripleBuffer* _tb;
    unsigned int _count;
};

class ReadThread : public OpenThreads::Thread
{
public:
    ReadThread( osgbBullet::TripleBuffer* tb )
      : _tb( tb )
    {}

    virtual void run()
    {
        while( !testCancel() )
        {
            char* addr = _tb->beginRead();
            if( addr == NULL )
                continue;

            unsigned int* iAddr = reinterpret_cast< unsigned int* >( addr );
            std::cout << _timer.time_s() << ": " << *iAddr << std::endl;
            _tb->endRead();
        }
    }

protected:
    osgbBullet::TripleBuffer* _tb;
    osg::Timer _timer;
};



int
main( int argc, char * argv[] )
{
    osgbBullet::TripleBuffer tb;
    WriteThread wt( &tb );
    ReadThread rt( &tb );

    wt.start();
    rt.start();

    osg::Timer timer;
    while( timer.time_s() < 2.0 )
    {
        OpenThreads::Thread::microSleep( 100 );
    }

    rt.cancel();
    wt.cancel();
    rt.join();
    wt.join();
}
