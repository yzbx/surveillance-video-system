#ifndef DATADRIVEFUNCTIONS_H
#define DATADRIVEFUNCTIONS_H

#include "DataDriveMain.h"
#include <memory>
namespace DataDrive {
class Base
{
public:
    Base(std::shared_ptr<DataDriveMain> data){
        this->data=data;
    }

    virtual bool run(){
        assert(false);
        return true;
    }

    virtual ~Base(){}
    std::shared_ptr<DataDriveMain> data;
};

class Input:public Base{
public:
    Input(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class Bgs:public Base{
public:
    Bgs(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class BlobFeature:public Base{
public:
    BlobFeature(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class Tracker:public Base{
public:
    Tracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class KLTTracker:public Base{
public:
    KLTTracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

}
#endif // DATADRIVEFUNCTIONS_H
