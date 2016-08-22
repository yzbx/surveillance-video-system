#ifndef CVHOGDETECTOR_H
#define CVHOGDETECTOR_H
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <iostream>

using namespace cv;
using namespace std;

class CVHOGDetector
{
public:
    CVHOGDetector(int argc,char *argv[]);
    int main(int argc, char **argv);
};

#endif // CVHOGDETECTOR_H
