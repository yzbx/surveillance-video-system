#ifndef DATADRIVEFUNCTIONS_H
#define DATADRIVEFUNCTIONS_H

#include "DataDriveFunction001.h"
#include "DataDriveMain.h"
#include "yzbx_config.h"
#include <SVS-plugin/ReDetection.h>
#include <memory>

namespace DataDrive {

class Base
{
public:
    GET_CLASS_NAME
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
    GET_CLASS_NAME
    Input(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class Bgs:public Base{
public:
    GET_CLASS_NAME
    Bgs(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class BlobFeature:public Base{
public:
    GET_CLASS_NAME
    BlobFeature(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class Tracker:public Base{
public:
    GET_CLASS_NAME
    Tracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class KLTTracker:public Base{
public:
    GET_CLASS_NAME
    KLTTracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};


class MeanShiftTracker:public Base{
public:
    GET_CLASS_NAME
    MeanShiftTracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class CamShiftTracker:public Base{
public:
    GET_CLASS_NAME
    CamShiftTracker(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class KLTAssignment:public Base{
public:
    GET_CLASS_NAME
    KLTAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
    void dump();
};

class OverLapAssignment:public Base{
public:
    GET_CLASS_NAME
    OverLapAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class RestOverLapAssignment:public Base{
public:
    GET_CLASS_NAME
    RestOverLapAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class HungarianAssignment:public Base{
public:
    GET_CLASS_NAME
    HungarianAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    track_t calcDist(std::shared_ptr<trackingObjectFeature> of1,
                     std::shared_ptr<trackingObjectFeature> of2,int costType=PosDist);
    bool run();
    void dump(assignments_t unMatchedAssignment,vector<Index_t> unMatchedObjects,vector<Index_t> unMatchedBlobs);
};

class ShowAssignment:public Base{
public:
    GET_CLASS_NAME
    ShowAssignment(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};


class SplitAndMerge:public Base{
public:
    GET_CLASS_NAME
    SplitAndMerge(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
    void handleOneToOneObjects();
    void handleNewObjects();
    void handleMissedObjects();
    // split first, then merge!!!
    void handleOneToNObjects();
    // merge later, split first!!!
    void handleNToOneObjects();
    //if find correspond track A, erase A from missed track set, normal update track A.
    //else do nothing
    bool redetection(Index_t newBlobIdx);
    //transform of.rect and of.pos, return true; else do nothing and return false.
    bool AffineTransform(const Index_t blobIdx,const Index_t trackIdx,trackingObjectFeature &of);
    void dump();
};

class FilterDeleteObjectToDump:public Base{
public:
    GET_CLASS_NAME
    FilterDeleteObjectToDump(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
private:
    bool isGoodObject(Id_t id){
        (void)id;
        return true;
    }
    void dumpTracjectoryAfterFilter(int trackIdx);
};

class FilterBadTrack:public Base{
public:
    GET_CLASS_NAME
    FilterBadTrack(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
};

class SaveToVideo:public Base{
public:
    GET_CLASS_NAME
    SaveToVideo(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();
    bool firstTime=true;
    string SaveVideoName;
    VideoWriter videoWriter;
};

class Countor:public Base{
public:
    GET_CLASS_NAME
    Countor(std::shared_ptr<DataDriveMain> data):Base(data){}
    bool run();

    Point_t center;
    double angle=0;

    bool isLineInited=false;
    void initLine();
    void getABC(double &A,double &B,double &C);
    void getBoundaryPoint(Point &p1, Point &p2);
    void dump();
    int count=0;
};

}
#endif // DATADRIVEFUNCTIONS_H
