#ifndef DATADRIVETRACKER_H
#define DATADRIVETRACKER_H
#include "DataDrivePipeLine.h"

class DataDriveTracker:public DataDrivePipeLine
{
public:
    DataDriveTracker(QString configFile);
    void run();
};

#endif // DATADRIVETRACKER_H
