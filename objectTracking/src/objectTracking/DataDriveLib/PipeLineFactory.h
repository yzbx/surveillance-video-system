#ifndef PIPELINEFACTORY_H
#define PIPELINEFACTORY_H
#include "DataDrivePipeLine.h"

class PipeLineFactory:public DataDrivePipeLine
{
public:
    PipeLineFactory(QString configFile);
    void run();
    void runAll();
    std::shared_ptr<DataDrive::Base> stringToClass(string classname);
    void loadPipeLineConfig();
};

#endif // PIPELINEFACTORY_H
