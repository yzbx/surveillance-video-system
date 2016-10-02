#include "DataDrivePipeLine.h"

DataDrivePipeLine::DataDrivePipeLine(QString configFile)
{
    this->mainData=std::make_shared<DataDriveMain>(DataDriveMain(configFile));
}

void DataDrivePipeLine::run()
{
    assert(false);
}

void DataDrivePipeLine::runAll()
{
    assert(false);
}
