#ifndef PIPELINETRACKING_H
#define PIPELINETRACKING_H
#include <QtCore>
#include "frameinput.h"
#include "../lib/bgsfactory_yzbx.h"
#include "BlobDetector.h"
#include "RectFloatTracker.h"
#include "TrackingBlobsMatchAnnotation.h"
#include "TrackingResultReplay.h"

class PipeLineTracking
{
public:
    PipeLineTracking();
    void process(QString sourceData, QString annTxt);
    void process(QString sourceData);
    void PipeLine_Input(QString sourceData);
    void PipeLine_Bgs();
    void PipeLine_Features(std::vector<trackingObjectFeature> &fv);
//    void PipeLine_Tracking();
    void PipeLine_DumpFV(int frameNum, const vector<int> &ids, vector<trackingObjectFeature> &fv);
    void PipeLine_Replay(QString dataSource, QString replaySource="",bool saveVideo=true);
    void setBgsType(QString bgsType){
        this->bgsType=bgsType;
    }
    void setRecordFile(QString recordFile);
private:
    QString recordFile;
    bool firstDump=true;
    track_t dist_thres=200;

    QString bgsType="SJN_MultiCueBGS";
//    QString bgsType="LOBSTERBGS";
    FrameInput fin;
    cv::Mat img_input,img_fg,img_bg;
    bgsFactory_yzbx fac;
    IBGS *ibgs=NULL;
    BlobDetector bd;
    RectFloatTracker tracker;
    TrackingBlobsMatchAnnotation matcher;

};

#endif // PIPELINETRACKING_H
