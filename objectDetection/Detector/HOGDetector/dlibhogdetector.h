#ifndef DLIBHOGDETECTOR_H
#define DLIBHOGDETECTOR_H
#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>

#include <iostream>
#include <fstream>


using namespace std;
using namespace dlib;


class DlibHOGDetector
{
public:
    DlibHOGDetector(int argc,char *argv[]);
    int main(int argc,char **argv);
};

#endif // DLIBHOGDETECTOR_H
