#ifndef OBJECTCOUNT_H
#define OBJECTCOUNT_H
#include "yzbx_utility.h"
#include "DataDrivePipeLine.h"

class ObjectCount : public DataDrivePipeLine
{
public:
    ObjectCount(QString configFile);
    void run();
};

#endif // OBJECTCOUNT_H
