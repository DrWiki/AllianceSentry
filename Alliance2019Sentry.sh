#!/bin/bash
echo "Hello Sentry!"
sleep 1s
echo 'nvidia' | sudo -S chmod 777 /dev/ttyUSB0
sleep 1s
cd  ~/ClionHome_/Alliance2019_Sentry/cmake-build-debug/
./Alliance2019_Sentry




