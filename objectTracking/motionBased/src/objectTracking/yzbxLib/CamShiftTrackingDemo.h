#ifndef CAMSHIFTTRACKINGDEMO_H
#define CAMSHIFTTRACKINGDEMO_H
#include "DataDrivePipeLine.h"

class CamShiftTrackingDemo:public DataDrivePipeLine
{
public:
    CamShiftTrackingDemo(QString configFile);
    void run();
};

#endif // CAMSHIFTTRACKINGDEMO_H
