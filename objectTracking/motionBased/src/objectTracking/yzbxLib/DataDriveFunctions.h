#ifndef DATADRIVEFUNCTIONS_H
#define DATADRIVEFUNCTIONS_H

#include "DataDriveMain.h"
#include "yzbx_config.h"
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


class MeanShiftTracker:public Base{
public:
    MeanShiftTracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class CamShiftTracker:public Base{
public:
    CamShiftTracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class KLTAssignment:public Base{
public:
    KLTAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class HungarianAssignment:public Base{
public:
    HungarianAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    track_t calcDist(std::shared_ptr<trackingObjectFeature> of1,
                     std::shared_ptr<trackingObjectFeature> of2,int costType=PosDist);
    bool run();
};

class ShowAssignment:public Base{
public:
    ShowAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};


class SplitAndMerge:public Base{
public:
    SplitAndMerge(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
    void handleOneToOneObjects();
    void handleNewObjects();
    void handleMissedObjects();
    //NOTE split first, then merge!!!
    void handleOneToNObjects();
    //NOTE merge later, split first!!!
    void handleNToOneObjects();

};


class UpdateTrackStatus:public Base{
public:
    UpdateTrackStatus(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};
}
#endif // DATADRIVEFUNCTIONS_H
