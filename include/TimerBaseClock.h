#pragma once
#include<iostream>
class TimerBaseClock{
public:
    TimerBaseClock();
    void Clear();
    bool isStarted();
    void start();
    unsigned long GetMs();
private:
    clock_t m_start;
};
