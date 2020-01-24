#!/bin/bash

#g++ -I/usr/include/opencv4 videoLinedetection.cpp -o VLD -L /usr/local/lib -lopencv_core -lopencv_videoio -lopencv_highgui -lopencv_cudaimgproc -lopencv_imgproc


g++ -I/usr/include/opencv4 $1 -o $2 -L /usr/local/lib -lopencv_core -lopencv_videoio -lopencv_highgui -lopencv_cudaimgproc -lopencv_imgproc
chmod +x $2
