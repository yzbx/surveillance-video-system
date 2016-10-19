#ifndef DATADRIVEPIPELINE_H
#define DATADRIVEPIPELINE_H
#include "DataDriveMain.h"
#include "DataDriveFunctions.h"
#include "DataDriveFunction002.h"
#include <boost/any.hpp>
#include <iostream>
#include <memory>

class DataDrivePipeLine
{
public:
    DataDrivePipeLine(QString configFile);
    virtual void run();
    virtual void runAll();

//private:
    std::shared_ptr<DataDriveMain> mainData;
    std::vector<std::shared_ptr<DataDrive::Base>> pipeline;
};

#endif // DATADRIVEPIPELINE_H
