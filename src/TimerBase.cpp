#ifndef TIMERBASE_H_
#define TIMERBASE_H_
#include"TimerBase.h"


TimerBase::TimerBase() : m_start(std::chrono::system_clock::time_point::min()){

}

void TimerBase::Clear(){
    m_start = std::chrono::system_clock::time_point::min();
}

bool TimerBase::isStarted() const{
    return (m_start.time_since_epoch()!=std::chrono::system_clock::duration(0));
}

void TimerBase::Start(){
    m_start = std::chrono::system_clock::now();
}

unsigned long TimerBase::GetMs(){
    if(isStarted()){
        std::chrono::system_clock::duration diff;
        diff = std::chrono::system_clock::now() - m_start;
        return (unsigned)(std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
    }
    return 0;
}

#endif
