#ifndef KLTTRACKINGDEMO_H
#define KLTTRACKINGDEMO_H
#include <QtCore>
#include "DataDriveMain.h"
#include "DataDrivePipeLine.h"
class KLTTrackingDemo : public DataDrivePipeLine
{
public:
    KLTTrackingDemo(QString configFile);
    void run();
};

#endif // KLTTRACKINGDEMO_H
