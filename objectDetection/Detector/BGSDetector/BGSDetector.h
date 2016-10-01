#ifndef BGSDETECTOR_H
#define BGSDETECTOR_H
//#include "../../../objectTracking/motionBased/src/objectTracking/yzbxLib/qyzbxlib.h"
#include "qyzbxlib.h"

class BGSDetector:public DataDrivePipeLine
{
public:
    BGSDetector(QString configFile);
    void run();
    void runAll();
};

#endif // BGSDETECTOR_H
