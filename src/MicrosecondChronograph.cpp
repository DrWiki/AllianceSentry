//
// Created by nvidia on 19-1-17.
//
#include "MicrosecondChronograph.h"
MicrosecondChronograph::MicrosecondChronograph() {
    t1 = 0;
    t2 = 0;
}



void MicrosecondChronograph::Start() {
    t1 = clock();
}

void MicrosecondChronograph::End() {                       // used in a loop
    t2 = clock();
    std::cout<<"## "<<"Spend " <<(static_cast<double>(t2)-static_cast<double>(t1))/55000<<std::endl;
}

void MicrosecondChronograph::End(std::string s) {          // used in a loop
    t2 = clock();
    std::cout<<"## "<<"Spend " <<(static_cast<double>(t2)-static_cast<double>(t1))/55000<<std::endl;
}

void MicrosecondChronograph::EndAndStart() {               // used in a period
    t2 = clock();
    std::cout<<"## "<<"Spend " <<(static_cast<double>(t2)-static_cast<double>(t1))/55000<<std::endl;
    t1 = t2;
}

void MicrosecondChronograph::EndAndStart(std::string s) {  //used in a period
    t2 = clock();
    std::cout<<"## "<<"Spend " <<(static_cast<double>(t2)-static_cast<double>(t1))/55000<<std::endl;
    t1 = t2;
}
