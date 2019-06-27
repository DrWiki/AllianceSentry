#pragma once
#include"TimerBaseClock.h"
TimerBaseClock::TimerBaseClock()
{
    m_start = -1;
}

void TimerBaseClock::Clear()
{
    m_start =-1;
}

bool TimerBaseClock::isStarted()
{
    return (m_start != -1);
}

void TimerBaseClock::start()
{
    m_start = clock();
}

unsigned long TimerBaseClock::GetMs()
{
    clock_t now;
    if(isStarted()){
        now = clock();
        clock_t dt = (now - m_start);
        return (unsigned long)(dt * 1000/CLOCKS_PER_SEC);
    }
    return 0;
}


