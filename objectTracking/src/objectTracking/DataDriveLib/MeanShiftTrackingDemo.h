#ifndef MEANSHIFTTRACKINGDEMO_H
#define MEANSHIFTTRACKINGDEMO_H
#include "DataDrivePipeLine.h"

class MeanShiftTrackingDemo: public DataDrivePipeLine
{
public:
    MeanShiftTrackingDemo(QString configFile);
    void run();
};

#endif // MEANSHIFTTRACKINGDEMO_H
