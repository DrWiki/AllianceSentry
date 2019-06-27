#pragma once
#include<iostream>
#include<chrono>
class TimerBase{
public:
    TimerBase();
    void Clear();
    bool isStarted() const;
    void Start();
    unsigned long GetMs();
private:
    std::chrono::system_clock::time_point m_start;
};
