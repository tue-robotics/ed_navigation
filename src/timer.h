//////////////////////////////////////////////////////////////////////////////
// Timer.h
// =======
// High Resolution Timer.
// This timer is able to measure the elapsed time with 1 micro-second accuracy
// in both Windows, Linux and Unix system 
//
//  AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// CREATED: 2003-01-13
// UPDATED: 2006-01-13
//
// Copyright (c) 2003 Song Ho Ahn
//////////////////////////////////////////////////////////////////////////////

#ifndef TIMER_H_DEF
#define TIMER_H_DEF

#ifdef WIN32   // Windows system specific
#include <windows.h>
#else          // Unix based system specific
#include <sys/time.h>
#endif

#include <iostream>
#include <string>

class Timer
{
public:

// ----------------------------------------------------------------------------------------------------

    Timer()
    {
#ifdef WIN32
        QueryPerformanceFrequency(&frequency);
        startCount.QuadPart = 0;
        endCount.QuadPart = 0;
#else
        start_count_.tv_sec = start_count_.tv_usec = 0;
        end_count_.tv_sec = end_count_.tv_usec = 0;
#endif

        stopped = 0;
        reset();
    }

// ----------------------------------------------------------------------------------------------------

    ~Timer() {}

// ----------------------------------------------------------------------------------------------------

    void reset()
    {
        stopped = 0; // reset stop flag
#ifdef WIN32
        QueryPerformanceCounter(&startCount);
#else
        gettimeofday(&start_count_, NULL);
#endif
    }

// ----------------------------------------------------------------------------------------------------

    void stop()
    {
        stopped = 1; // set timer stopped flag

#ifdef WIN32
        QueryPerformanceCounter(&endCount);
#else
        gettimeofday(&end_count_, NULL);
#endif
    }

// ----------------------------------------------------------------------------------------------------

    double getElapsedTimeInSec() const
    {
        return this->getElapsedTimeInMicroSec() * 0.000001;
    }

    double getElapsedTimeInMilliSec() const
    {
        return this->getElapsedTimeInMicroSec() * 0.001;
    }

// ----------------------------------------------------------------------------------------------------

    double getElapsedTimeInMicroSec() const
    {
#ifdef WIN32
        if(!stopped)
            QueryPerformanceCounter(&endCount);

        double startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
        double endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
        timeval end_count;
        if (stopped) {
            end_count = end_count_;
        } else {
            gettimeofday(&end_count, NULL);
        }

        double startTimeInMicroSec = (start_count_.tv_sec * 1000000.0) + start_count_.tv_usec;
        double endTimeInMicroSec = (end_count.tv_sec * 1000000.0) + end_count.tv_usec;
#endif

        return endTimeInMicroSec - startTimeInMicroSec;
    }

// ----------------------------------------------------------------------------------------------------

private:
    //    double startTimeInMicroSec;                 // starting time in micro-second
    //    double endTimeInMicroSec;                   // ending time in micro-second
    int    stopped;                             // stop flag
#ifdef WIN32
    LARGE_INTEGER frequency;                    // ticks per second
    LARGE_INTEGER startCount;                   //
    LARGE_INTEGER endCount;                     //
#else
    timeval start_count_;                         //
    timeval end_count_;                           //
#endif
};

// ----------------------------------------------------------------------------------------------------

#endif // TIMER_H_DEF
