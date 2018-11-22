#!/bin/bash

opencv_createsamples -img img.jpg -bg bg.txt -info info/info.lst -pngoutput info -maxxangle 0.5 -maxyangle 0.5 -maxzangle 0.5 -num 2451
opencv_createsamples -info info/info.lst -num 2451 -w 20 -h 20 -vec positives.vec
opencv_traincascade -data data -vec positives.vec -bg bg.txt -numPos 1800 -numNeg 900 -numStages 10 -w 20 -h 20