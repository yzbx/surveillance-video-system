#ifndef SINGLEOBJECTTRACKER_H
#define SINGLEOBJECTTRACKER_H
#include <opencv2/opencv.hpp>
#include <QtCore>
#include "trackingobjectfeature.h"
#include "Kalman.h"
#include "yzbx_config.h"
#include "yzbx_utility.h"

class singleObjectTracker
{
public:
    singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID,int frameNum=0,
    int img_width=std::numeric_limits<int>::max(),int img_height=std::numeric_limits<int>::max());
    int img_width;
    int img_height;
    TKalmanFilter KF;
    int track_id;
    std::vector<Point_t> trace;
    std::vector<Rect_t> rects;
    std::vector<STATUS> vec_status;
    std::vector<int> frames;
    Point_t prediction;
    std::unique_ptr<trackingObjectFeature> feature;
    STATUS status;
    Point_t firstSeePos;
    Point_t lastSeePos;
    SplitMergeType splitMergeType;
    //update when use normal update!!!
    cv::Mat img_last_normal;
    cv::Mat mask_last_normal;
    Rect_t rect_last_normal;
private:
    int skipped_frames;
    int catch_frames;
    int lifetime;
    bool AvoidUpdate;
public:
    int get_skipped_frames(){
        return skipped_frames;
    }
    int get_catch_frames(){
        return catch_frames;
    }
    int get_lifetime(){
        return lifetime;
    }

    void set_skipped_frames(int a){
        skipped_frames=a;
    }
    void set_catch_frames(int a){
        catch_frames=a;
    }
    void set_lifetime(int a){
        lifetime=a;
    }

    track_t CalcDist(trackingObjectFeature &of);
    void Update(const trackingObjectFeature &of, bool dataCorrect, size_t max_trace_length);
    void NormalUpdate(const trackingObjectFeature &of, int frameNum, const Mat &img_normal=cv::Mat());
    void MissUpdate(int frameNum);
    void OneToNUpdateForBiggestBlob(const trackingObjectFeature &of,int frameNum);
    void OneToNUpdateForNewBlob();
    void NToOneUpdate(trackingObjectFeature &of,int frameNum);
    void PreUpdateForBiggestBlob(const trackingObjectFeature &of);
    void AvoidUpdateTwice();
    Rect_t GetPredictRect();
    void dumpToScreen(){
        map<STATUS,string> statusStr;
        statusStr[MISSING_STATUS]="missing";
        statusStr[NORMAL_STATUS]="normal";
        statusStr[NEW_STATUS]="new";
        statusStr[OneToN_STATUS]="OneToN";
        statusStr[NToOne_STATUS]="NToOne";
//        DELETE_TO_SPLIT,DELETE_TO_MERGE,PREUPDATE_STATUS
        statusStr[DELETE_TO_SPLIT]="delete_split";
        statusStr[DELETE_TO_MERGE]="delete_merge";
        statusStr[PREUPDATE_STATUS]="preupdate";
        cout<<"track_id="<<track_id<<endl;
        cout<<"lifetime="<<lifetime;
        cout<<", onBoudary="<<feature->onBoundary;
        int frameNum=frames.back();
        cout<<", frameNum="<<frameNum;
//        cout<<", skipped_frame="<<skipped_frames;
//        cout<<", catch_frames="<<catch_frames;
        cout<<", status="<<statusStr[status];

//        cout<<", frames=(";
//        for(uint i=0;i<frames.size();i++){
//            cout<<frames[i]<<",";
//        }
//        cout<<")";

//        cout<<", firstSeePos=("<<firstSeePos.x<<","<<firstSeePos.y<<")";
//        cout<<", lastSeePos=("<<lastSeePos.x<<","<<lastSeePos.y<<")";
        cout<<", displacement="<<cv::norm(lastSeePos-firstSeePos);
        cout<<", rect=("<<feature->rect.x<<","<<feature->rect.y
           <<","<<feature->rect.width<<","<<feature->rect.height<<")";

        cout<<endl;
    }

    QString dump(){
        QStringList dumpstrs;
        dumpstrs<<QString::number(track_id)
               <<QString::number(status)
               <<QString::number(prediction.x)
               <<QString::number(prediction.y)
               <<QString::number(skipped_frames)
              <<feature->dump();
        return dumpstrs.join("\t");
    }
    void predict(trackingObjectFeature &of);
};

#endif // SINGLEOBJECTTRACKER_H
