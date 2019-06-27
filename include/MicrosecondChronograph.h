//
// Created by nvidia on 19-1-17.
//

#ifndef ALLIANCE2019_SENTRY_MICROSECONDCHRONOGRAPH_H
#define ALLIANCE2019_SENTRY_MICROSECONDCHRONOGRAPH_H

#include <ctime>
#include <iostream>
class MicrosecondChronograph {
public:
    clock_t t1 = 0;
    clock_t t2 = 0;
    MicrosecondChronograph();
    void Start();
    void End();
    void End(std::string);
    void EndAndStart();
    void EndAndStart(std::string);
};


#endif //ALLIANCE2019_SENTRY_MICROSECONDCHRONOGRAPH_H
