#ifndef B2BTRACKINGDEMO_H
#define B2BTRACKINGDEMO_H
#include "DataDrivePipeLine.h"

class B2BTrackingDemo:public DataDrivePipeLine
{
public:
    B2BTrackingDemo(QString configFile);
    void run();
    void runAll();
};

#endif // B2BTRACKINGDEMO_H
